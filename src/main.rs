//! A shader that renders a mesh multiple times in one draw call.

mod orbbec;

use bevy::{
    core_pipeline::core_3d::Transparent3d,
    ecs::{
        query::QueryItem,
        system::{lifetimeless::*, SystemParamItem},
    },
    pbr::{
        MeshPipeline, MeshPipelineKey, RenderMeshInstances, SetMeshBindGroup, SetMeshViewBindGroup,
    },
    prelude::*,
    render::{
        extract_component::{ExtractComponent, ExtractComponentPlugin},
        mesh::{GpuBufferInfo, GpuMesh},
        render_asset::RenderAssets,
        render_phase::{
            AddRenderCommand, DrawFunctions, PhaseItem, RenderCommand,
            RenderCommandResult, SetItemPipeline, TrackedRenderPass,
        },
        render_resource::*,
        renderer::RenderDevice,
        view::{ExtractedView, NoFrustumCulling},
        Render, RenderApp, RenderSet,
    },
};
use bytemuck::{Pod, Zeroable};
use orbbec_sdk::ob;
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;
use bevy::render::mesh::MeshVertexBufferLayout;
use bevy::render::render_phase::RenderPhase;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use crate::orbbec::OrbbecRx;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, CustomMaterialPlugin))
        .add_systems(Startup, setup)
        .add_systems(Update, update)
        .init_resource::<OrbbecRx>()
        .run();
}

fn setup(mut commands: Commands) {
    // camera
    commands.spawn((Camera3dBundle {
        transform: Transform::from_xyz(0.0, 0.0, 1500.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    },     PanOrbitCamera::default(),));
}

fn screen_to_ndc(screen_coords: Vec2, window: &Window) -> Vec3 {
    Vec3::new(
        (screen_coords.x / window.width() as f32) * 2.0 - 1.0,
        (screen_coords.y / window.height() as f32) * 2.0 - 1.0,
        1.0, // Assuming depth is at the far plane
    )
}

fn screen_to_ndc_with_depth(screen_coords: Vec2, depth: f32, window: &Window) -> Vec3 {
    Vec3::new(
        (screen_coords.x / window.width() as f32) * 2.0 - 1.0,
        (screen_coords.y / window.height() as f32) * 2.0 - 1.0,
        depth, // Use your depth value here, normalized if necessary
    )
}

fn ndc_to_world(ndc_coords: Vec3, camera: &Camera, camera_transform: &GlobalTransform) -> Vec3 {
    let view_matrix = camera_transform.compute_matrix();
    let projection_matrix = camera.projection_matrix();
    let view_projection_inverse = projection_matrix * view_matrix.inverse();

    let world_coords = view_projection_inverse.project_point3(ndc_coords);

    world_coords
}

fn update(
    mut commands: Commands,
    instance: Query<Entity, With<InstanceMaterialData>>,
    mut meshes: ResMut<Assets<Mesh>>,
    orbbec: Res<OrbbecRx>,
) {
    if let Some(data) = orbbec.try_get_data() {
        let instance_data = InstanceMaterialData(
            data.iter()
                .map(|point| {
                    let color = Color::rgb(point.r, point.g, point.b);
                    InstanceData {
                        position: Vec3::new(point.x, point.y, point.z),
                        scale: 1.0,
                        color: color.as_linear_rgba_f32(),
                    }
                })
                .collect(),
        );

        if let Some(entity) = instance.iter().next() {
            commands.entity(entity).insert(instance_data);
        } else {
            commands.spawn((
                meshes.add(Cuboid::new(0.5, 0.5, 0.5)),
                SpatialBundle::INHERITED_IDENTITY,
                instance_data,
                // NOTE: Frustum culling is done based on the Aabb of the Mesh and the GlobalTransform.
                // As the cube is at the origin, if its Aabb moves outside the view frustum, all the
                // instanced cubes will be culled.
                // The InstanceMaterialData contains the 'GlobalTransform' information for this custom
                // instancing, and that is not taken into account with the built-in frustum culling.
                // We must disable the built-in frustum culling by adding the `NoFrustumCulling` marker
                // component to avoid incorrect culling.
                NoFrustumCulling,
            ));
        }
    }
}

#[derive(Component, Deref)]
struct InstanceMaterialData(Vec<InstanceData>);

impl ExtractComponent for InstanceMaterialData {
    type QueryData = &'static InstanceMaterialData;
    type QueryFilter = ();
    type Out = Self;

    fn extract_component(item: QueryItem<'_, Self::QueryData>) -> Option<Self> {
        Some(InstanceMaterialData(item.0.clone()))
    }
}

struct CustomMaterialPlugin;

impl Plugin for CustomMaterialPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(ExtractComponentPlugin::<InstanceMaterialData>::default())
            .add_plugins(PanOrbitCameraPlugin);

        app.sub_app_mut(RenderApp)
            .add_render_command::<Transparent3d, DrawCustom>()
            .init_resource::<SpecializedMeshPipelines<CustomPipeline>>()
            .add_systems(
                Render,
                (
                    queue_custom.in_set(RenderSet::QueueMeshes),
                    prepare_instance_buffers.in_set(RenderSet::PrepareResources),
                ),
            );
    }

    fn finish(&self, app: &mut App) {
        app.sub_app_mut(RenderApp).init_resource::<CustomPipeline>();
    }
}
#[derive(Clone, Copy, Pod, Zeroable)]
#[repr(C)]
struct InstanceData {
    position: Vec3,
    scale: f32,
    color: [f32; 4],
}

#[allow(clippy::too_many_arguments)]
fn queue_custom(
    transparent_3d_draw_functions: Res<DrawFunctions<Transparent3d>>,
    custom_pipeline: Res<CustomPipeline>,
    msaa: Res<Msaa>,
    mut pipelines: ResMut<SpecializedMeshPipelines<CustomPipeline>>,
    pipeline_cache: Res<PipelineCache>,
    meshes: Res<RenderAssets<Mesh>>,
    render_mesh_instances: Res<RenderMeshInstances>,
    material_meshes: Query<Entity, With<InstanceMaterialData>>,
    mut views: Query<(&ExtractedView, &mut RenderPhase<Transparent3d>)>,
) {
    let draw_custom = transparent_3d_draw_functions.read().id::<DrawCustom>();

    let msaa_key = MeshPipelineKey::from_msaa_samples(msaa.samples());

    for (view, mut transparent_phase) in &mut views {
        let view_key = msaa_key | MeshPipelineKey::from_hdr(view.hdr);
        let rangefinder = view.rangefinder3d();
        for entity in &material_meshes {
            let Some(mesh_instance) = render_mesh_instances.get(&entity) else {
                continue;
            };
            let Some(mesh) = meshes.get(mesh_instance.mesh_asset_id) else {
                continue;
            };
            let key = view_key | MeshPipelineKey::from_primitive_topology(mesh.primitive_topology);
            let pipeline = pipelines
                .specialize(&pipeline_cache, &custom_pipeline, key, &mesh.layout)
                .unwrap();
            transparent_phase.add(Transparent3d {
                entity,
                pipeline,
                draw_function: draw_custom,
                distance: rangefinder
                    .distance_translation(&mesh_instance.transforms.transform.translation),
                batch_range: 0..1,
                dynamic_offset: None,
            });
        }
    }
}

#[derive(Component)]
struct InstanceBuffer {
    buffer: Buffer,
    length: usize,
}

fn prepare_instance_buffers(
    mut commands: Commands,
    query: Query<(Entity, &InstanceMaterialData)>,
    render_device: Res<RenderDevice>,
) {
    for (entity, instance_data) in &query {
        let buffer = render_device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("instance data buffer"),
            contents: bytemuck::cast_slice(instance_data.as_slice()),
            usage: BufferUsages::VERTEX | BufferUsages::COPY_DST,
        });
        commands.entity(entity).insert(InstanceBuffer {
            buffer,
            length: instance_data.len(),
        });
    }
}

#[derive(Resource)]
struct CustomPipeline {
    shader: Handle<Shader>,
    mesh_pipeline: MeshPipeline,
}

impl FromWorld for CustomPipeline {
    fn from_world(world: &mut World) -> Self {
        let asset_server = world.resource::<AssetServer>();
        let shader = asset_server.load("shaders/instancing.wgsl");

        let mesh_pipeline = world.resource::<MeshPipeline>();

        CustomPipeline {
            shader,
            mesh_pipeline: mesh_pipeline.clone(),
        }
    }
}

impl SpecializedMeshPipeline for CustomPipeline {
    type Key = MeshPipelineKey;

    fn specialize(
        &self,
        key: Self::Key,
        layout: &MeshVertexBufferLayout,
    ) -> Result<RenderPipelineDescriptor, SpecializedMeshPipelineError> {
        let mut descriptor = self.mesh_pipeline.specialize(key, layout)?;

        descriptor.vertex.shader = self.shader.clone();
        descriptor.vertex.buffers.push(VertexBufferLayout {
            array_stride: std::mem::size_of::<InstanceData>() as u64,
            step_mode: VertexStepMode::Instance,
            attributes: vec![
                VertexAttribute {
                    format: VertexFormat::Float32x4,
                    offset: 0,
                    shader_location: 3, // shader locations 0-2 are taken up by Position, Normal and UV attributes
                },
                VertexAttribute {
                    format: VertexFormat::Float32x4,
                    offset: VertexFormat::Float32x4.size(),
                    shader_location: 4,
                },
            ],
        });
        descriptor.fragment.as_mut().unwrap().shader = self.shader.clone();
        Ok(descriptor)
    }
}

type DrawCustom = (
    SetItemPipeline,
    SetMeshViewBindGroup<0>,
    SetMeshBindGroup<1>,
    DrawMeshInstanced,
);

struct DrawMeshInstanced;

impl<P: PhaseItem> RenderCommand<P> for DrawMeshInstanced {
    type Param = (SRes<RenderAssets<Mesh>>, SRes<RenderMeshInstances>);
    type ViewQuery = ();
    type ItemQuery = Read<InstanceBuffer>;

    #[inline]
    fn render<'w>(
        item: &P,
        _view: (),
        instance_buffer: Option<&'w InstanceBuffer>,
        (meshes, render_mesh_instances): SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let Some(mesh_instance) = render_mesh_instances.get(&item.entity()) else {
            return RenderCommandResult::Failure;
        };
        let Some(gpu_mesh) = meshes.into_inner().get(mesh_instance.mesh_asset_id) else {
            return RenderCommandResult::Failure;
        };
        let Some(instance_buffer) = instance_buffer else {
            return RenderCommandResult::Failure;
        };

        pass.set_vertex_buffer(0, gpu_mesh.vertex_buffer.slice(..));
        pass.set_vertex_buffer(1, instance_buffer.buffer.slice(..));

        match &gpu_mesh.buffer_info {
            GpuBufferInfo::Indexed {
                buffer,
                index_format,
                count,
            } => {
                pass.set_index_buffer(buffer.slice(..), 0, *index_format);
                pass.draw_indexed(0..*count, 0, 0..instance_buffer.length as u32);
            }
            GpuBufferInfo::NonIndexed => {
                pass.draw(0..gpu_mesh.vertex_count, 0..instance_buffer.length as u32);
            }
        }
        RenderCommandResult::Success
    }
}