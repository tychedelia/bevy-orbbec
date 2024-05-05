use std::ffi::c_int;
use std::process::exit;
use std::ptr::null_mut;
use std::sync::{Arc, Mutex};
use std::sync::mpsc::{Receiver, Sender, SyncSender};
use std::thread::JoinHandle;

use bevy::prelude::{info, Resource};
use orbbec_sdk::{OBColorPoint, OBSensorType_OB_SENSOR_COLOR};
pub use orbbec_sdk::ob;

use crate::orbbec;

#[derive(Resource)]
pub struct OrbbecRx {
    jh: Option<JoinHandle<()>>,
    rx: Arc<Mutex<Receiver<Vec<OBColorPoint>>>>,
    pub tx_shutdown: SyncSender<()>,
}

impl Drop for OrbbecRx {
    fn drop(&mut self) {
        self.tx_shutdown.send(()).unwrap();
        self.jh.take().unwrap().join().unwrap();
    }
}

impl OrbbecRx {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn get_data(&self) -> Vec<ob::OBColorPoint> {
        self.rx.lock().unwrap().recv().unwrap()
    }

    pub fn try_get_data(&self) -> Option<Vec<ob::OBColorPoint>> {
        self.rx.lock().unwrap().try_recv().ok()
    }
}

impl Default for OrbbecRx {
    fn default() -> Self {
        let (tx, rx) = std::sync::mpsc::channel();
        let (tx_shutdown, rx_shutdown) = std::sync::mpsc::sync_channel(1);
        let jh = std::thread::spawn(|| unsafe {
            let mut orbbec = Orbbec::new();
            orbbec.run(tx, rx_shutdown);
        });

        Self {
            jh: Some(jh),
            tx_shutdown,
            rx: Arc::new(Mutex::new(rx)),
        }
    }
}

pub struct Orbbec {
    point_cloud: *mut ob::ob_filter,
    ob_pipeline: *mut ob::ob_pipeline,
    error: *mut ob::ob_error,
    ob_config: *mut ob::ob_config,
    depth_profile: *mut ob::ob_stream_profile,
    color_profile: *mut ob::ob_stream_profile,
    color_profiles: *mut ob::ob_stream_profile_list,
    depth_profiles: *mut ob::ob_stream_profile_list,
}

impl Orbbec {
    fn new() -> Self {
        Self {
            point_cloud: null_mut(),
            ob_pipeline: null_mut(),
            error: null_mut(),
            ob_config: null_mut(),
            depth_profile: null_mut(),
            color_profile: null_mut(),
            color_profiles: null_mut(),
            depth_profiles: null_mut(),
        }
    }

    unsafe fn check_error(&mut self) {
        if !self.error.is_null() {
            println!(
                "ob_error was raised:\n\tcall: {:?}({:?})",
                ob::ob_error_function(self.error),
                ob::ob_error_args(self.error),
            );
            let msg = ob::ob_error_message(self.error);
            let msg = std::ffi::CStr::from_ptr(msg).to_str().unwrap();
            println!("\tmessage: {:?}", msg);
            let msg = ob::ob_error_exception_type(self.error);
            println!("\terror type: {:?}", msg);
            ob::ob_delete_error(self.error);
            exit(1);
        }
    }

    pub unsafe fn run(&mut self, tx_points: Sender<Vec<OBColorPoint>>, rx_shutdown: Receiver<()>) {
        info!("Starting orbbec");

        ob::ob_set_logger_severity(ob::OBLogSeverity_OB_LOG_SEVERITY_ERROR, &mut self.error);
        self.check_error();

        self.ob_pipeline = ob::ob_create_pipeline(&mut self.error);
        self.check_error();

        // Create config to configure the resolution, frame rate, and format of Color and Depth streams
        self.ob_config = ob::ob_create_config(&mut self.error);
        self.check_error();

        info!("Config created");
        self.color_profiles = ob::ob_pipeline_get_stream_profile_list(
            self.ob_pipeline,
            OBSensorType_OB_SENSOR_COLOR,
            &mut self.error,
        );

        info!("color_profiles: {:?}", self.color_profiles);
        if !self.error.is_null() {
            println!("Current device is not support color sensor!");
            ob::ob_delete_error(self.error);
            self.error = null_mut();
            // Turn on D2C alignment, which needs to be turned on when generating RGBD point clouds
            ob::ob_config_set_align_mode(
                self.ob_config,
                ob::OBAlignMode_ALIGN_DISABLE,
                &mut self.error,
            );
            self.check_error();
        }

        // Open the default profile of Color Sensor, which can be configured through the configuration file
        if !self.color_profiles.is_null() {
            info!("Configuring default color profile");
            self.color_profile = ob::ob_stream_profile_list_get_profile(
                self.color_profiles,
                ob::OB_PROFILE_DEFAULT as c_int,
                &mut self.error,
            );
        }

        // enable stream
        if !self.color_profile.is_null() {
            ob::ob_config_enable_stream(self.ob_config, self.color_profile, &mut self.error);
            self.check_error();
        }

        // Configure depth flow
        let mut align_mode: ob::OBAlignMode = ob::OBAlignMode_ALIGN_DISABLE;

        info!("color_profile: {:?}", self.color_profile);
        if !self.color_profile.is_null() {
            // Try find supported depth to color align hardware mode profile
            self.depth_profiles = ob::ob_get_d2c_depth_profile_list(
                self.ob_pipeline,
                self.color_profile,
                ob::OBAlignMode_ALIGN_D2C_HW_MODE,
                &mut self.error,
            );
            self.check_error();
            let mut d2c_count =
                ob::ob_stream_profile_list_count(self.depth_profiles, &mut self.error);
            self.check_error();
            if d2c_count > 0 {
                align_mode = ob::OBAlignMode_ALIGN_D2C_HW_MODE;
            } else {
                // Try find supported depth to color align software mode profile
                self.depth_profiles = ob::ob_get_d2c_depth_profile_list(
                    self.ob_pipeline,
                    self.color_profile,
                    ob::OBAlignMode_ALIGN_D2C_SW_MODE,
                    &mut self.error,
                );
                self.check_error();
                d2c_count = ob::ob_stream_profile_list_count(self.depth_profiles, &mut self.error);
                self.check_error();
                if d2c_count > 0 {
                    align_mode = ob::OBAlignMode_ALIGN_D2C_SW_MODE;
                }
            }
        } else {
            self.depth_profiles = ob::ob_pipeline_get_stream_profile_list(
                self.ob_pipeline,
                ob::OBSensorType_OB_SENSOR_DEPTH,
                &mut self.error,
            );
            self.check_error();
        }

        let list_count = ob::ob_stream_profile_list_count(self.depth_profiles, &mut self.error);
        self.check_error();
        info!("list_count: {:?}", list_count);
        if list_count > 0 {
            if !self.color_profile.is_null() {
                info!("color_profile is not null");
                // Select the profile with the same frame rate as color.
                let color_fps =
                    ob::ob_video_stream_profile_fps(self.color_profile, &mut self.error);
                self.check_error();
                self.depth_profile = ob::ob_stream_profile_list_get_video_stream_profile(
                    self.depth_profiles,
                    ob::OB_WIDTH_ANY as c_int,
                    ob::OB_HEIGHT_ANY as c_int,
                    ob::OBFormat_OB_FORMAT_UNKNOWN,
                    color_fps as c_int,
                    &mut self.error,
                );
                self.check_error();
            }

            if self.depth_profile.is_null() {
                info!("depth_profile is null");
                // If no matching profile is found, select the default profile.
                self.depth_profile = ob::ob_stream_profile_list_get_profile(
                    self.depth_profiles,
                    ob::OB_PROFILE_DEFAULT as c_int,
                    &mut self.error,
                );
                self.check_error();
            }

            info!("enabling stream");
            // enable stream
            ob::ob_config_enable_stream(self.ob_config, self.depth_profile, &mut self.error);
            self.check_error();

            info!("align_mode: {:?}", align_mode);
            // Turn on D2C alignment, which needs to be turned on when generating RGBD point clouds
            ob::ob_config_set_align_mode(self.ob_config, align_mode, &mut self.error);
            self.check_error();
        }

        // Get the device handle
        let ob_device: *mut ob::ob_device =
            ob::ob_pipeline_get_device(self.ob_pipeline, &mut self.error);
        self.check_error();

        info!("Device: {:?}", ob_device);
        // Start the pipeline with config
        ob::ob_pipeline_start_with_config(self.ob_pipeline, self.ob_config, &mut self.error);
        self.check_error();

        info!("Pipeline started");
        // Create a point cloud Filter object (device parameters will be obtained inside the Pipeline when the point cloud filter is created, so try to configure
        // the device before creating the filter)
        self.point_cloud = ob::ob_create_pointcloud_filter(&mut self.error);
        self.check_error();

        info!("Point cloud filter created");
        // Obtain the current open-stream camera parameters from the pipeline and pass them to the point cloud filter
        let camera_param: ob::ob_camera_param =
            ob::ob_pipeline_get_camera_param(self.ob_pipeline, &mut self.error);
        self.check_error();
        ob::ob_pointcloud_filter_set_camera_param(self.point_cloud, camera_param, &mut self.error);
        self.check_error();

        let mut count = 0;
        let mut points_created = false;

        // Loop to get the frame and save the point cloud
        loop {
            if rx_shutdown.try_recv().is_ok() {
                break;
            }

            info!("count: {}", count);
            count = 0;
            // Limit up to 10 repetitions
            // Waiting for one frame, the timeout is 100ms
            info!("Waiting for frameset");
            let frameset: *mut ob::ob_frame =
                ob::ob_pipeline_wait_for_frameset(self.ob_pipeline, 100, &mut self.error);
            info!("Frameset: {:?}", frameset);
            self.check_error();
            if !frameset.is_null() {
                // get depth value scale
                info!("Getting depth frame");
                let depth_frame: *mut ob::ob_frame =
                    ob::ob_frameset_depth_frame(frameset, &mut self.error);
                self.check_error();
                if depth_frame.is_null() {
                    continue;
                }

                // get depth value scale
                info!("Getting depth value scale");
                let depth_value_scale: f32 =
                    ob::ob_depth_frame_get_value_scale(depth_frame, &mut self.error);
                self.check_error();

                // delete depth frame
                ob::ob_delete_frame(depth_frame, &mut self.error);
                self.check_error();

                // point position value multiply depth value scale to convert uint to millimeter (for some devices, the default depth value uint is not
                // millimeter)
                info!("Setting point position data scale");
                ob::ob_pointcloud_filter_set_position_data_scale(
                    self.point_cloud,
                    depth_value_scale,
                    &mut self.error,
                );
                self.check_error();

                info!("Setting point format");
                ob::ob_pointcloud_filter_set_point_format(
                    self.point_cloud,
                    ob::OBFormat_OB_FORMAT_RGB_POINT,
                    &mut self.error,
                );
                self.check_error();
                let points_frame: *mut ob::ob_frame =
                    ob::ob_filter_process(self.point_cloud, frameset, &mut self.error);
                info!("Points frame: {:?}", points_frame);
                self.check_error();
                if !points_frame.is_null() {
                    // PUSH
                    let points_size = ob::ob_frame_data_size(points_frame, &mut self.error)
                        as usize
                        / std::mem::size_of::<ob::OBColorPoint>();
                    self.check_error();

                    let points =
                        ob::ob_frame_data(points_frame, &mut self.error) as *mut ob::OBColorPoint;
                    let points = std::slice::from_raw_parts_mut(points, points_size);
                    self.check_error();

                    tx_points.send(points.to_vec()).unwrap();

                    ob::ob_delete_frame(points_frame, &mut self.error);
                    self.check_error();
                    points_created = true;
                }

                ob::ob_delete_frame(frameset, &mut self.error); // Destroy frameSet to reclaim memory
                self.check_error();
            }
        }
    }
}

impl Drop for Orbbec {
    fn drop(&mut self) {
        unsafe {
            ob::ob_delete_filter(self.point_cloud, &mut self.error);
            self.check_error();

            // stop pipeline
            ob::ob_pipeline_stop(self.ob_pipeline, &mut self.error);
            self.check_error();

            // destroy pipeline
            ob::ob_delete_pipeline(self.ob_pipeline, &mut self.error);
            self.check_error();

            // destroy config
            ob::ob_delete_config(self.ob_config, &mut self.error);
            self.check_error();

            // destroy profile
            ob::ob_delete_stream_profile(self.depth_profile, &mut self.error);
            self.check_error();

            // destroy profile
            ob::ob_delete_stream_profile(self.color_profile, &mut self.error);
            self.check_error();

            // destroy profile list
            ob::ob_delete_stream_profile_list(self.color_profiles, &mut self.error);
            self.check_error();

            // destroy profile list
            ob::ob_delete_stream_profile_list(self.depth_profiles, &mut self.error);
            self.check_error();
        }
    }
}
