pub use orbbec_sdk::ob;
use orbbec_sdk::{ob_color_point, OBSensorType_OB_SENSOR_COLOR};
use std::ffi::c_int;
use std::process::exit;
use std::ptr::{null_mut, slice_from_raw_parts_mut};

unsafe fn check_error(error: *mut ob::ob_error) {
    if !error.is_null() {
        println!(
            "ob_error was raised:\n\tcall: {:?}({:?})",
            ob::ob_error_function(error),
            ob::ob_error_args(error),
        );
        println!("\tmessage: {:?}", ob::ob_error_message(error));
        println!("\terror type: {:?}", ob::ob_error_exception_type(error));
        ob::ob_delete_error(error);
        exit(1);
    }
}

pub unsafe fn do_the_thing() {
    let mut error: *mut ob::ob_error = null_mut();
    let mut ob_pipeline: *mut ob::ob_pipeline = null_mut(); // pipeline, used to open the Color and Depth streams after connecting the device

    ob::ob_set_logger_severity(ob::OBLogSeverity_OB_LOG_SEVERITY_ERROR, &mut error);
    check_error(error);

    ob_pipeline = ob::ob_create_pipeline(&mut error);
    check_error(error);

    // Create config to configure the resolution, frame rate, and format of Color and Depth streams
    let ob_config: *mut ob::ob_config = ob::ob_create_config(&mut error);
    check_error(error);

    let mut color_profile: *mut ob::ob_stream_profile = null_mut();
    let color_profiles: *mut ob::ob_stream_profile_list = ob::ob_pipeline_get_stream_profile_list(
        ob_pipeline,
        OBSensorType_OB_SENSOR_COLOR,
        &mut error,
    );
    if !error.is_null() {
        println!("Current device is not support color sensor!");
        ob::ob_delete_error(error);
        error = null_mut();
        // Turn on D2C alignment, which needs to be turned on when generating RGBD point clouds
        ob::ob_config_set_align_mode(ob_config, ob::OBAlignMode_ALIGN_DISABLE, &mut error);
        check_error(error);
    }

    // Open the default profile of Color Sensor, which can be configured through the configuration file
    if !color_profile.is_null() {
        color_profile = ob::ob_stream_profile_list_get_profile(
            color_profiles,
            ob::OB_PROFILE_DEFAULT as c_int,
            &mut error,
        );
    }

    // enable stream
    if !color_profile.is_null() {
        ob::ob_config_enable_stream(ob_config, color_profile, &mut error);
        check_error(error);
    }

    // Configure depth flow
    let mut depth_profile: *mut ob::ob_stream_profile = null_mut();
    let mut align_mode: ob::OBAlignMode = ob::OBAlignMode_ALIGN_DISABLE;
    let mut depthProfiles: *mut ob::ob_stream_profile_list = null_mut();

    if !color_profile.is_null() {
        // Try find supported depth to color align hardware mode profile
        depthProfiles = ob::ob_get_d2c_depth_profile_list(
            ob_pipeline,
            color_profile,
            ob::OBAlignMode_ALIGN_D2C_HW_MODE,
            &mut error,
        );
        check_error(error);
        let mut d2cCount = ob::ob_stream_profile_list_count(depthProfiles, &mut error);
        check_error(error);
        if (d2cCount > 0) {
            align_mode = ob::OBAlignMode_ALIGN_D2C_HW_MODE;
        } else {
            // Try find supported depth to color align software mode profile
            depthProfiles = ob::ob_get_d2c_depth_profile_list(
                ob_pipeline,
                color_profile,
                ob::OBAlignMode_ALIGN_D2C_SW_MODE,
                &mut error,
            );
            check_error(error);
            d2cCount = ob::ob_stream_profile_list_count(depthProfiles, &mut error);
            check_error(error);
            if (d2cCount > 0) {
                align_mode = ob::OBAlignMode_ALIGN_D2C_SW_MODE;
            }
        }
    } else {
        depthProfiles = ob::ob_pipeline_get_stream_profile_list(
            ob_pipeline,
            ob::OBSensorType_OB_SENSOR_DEPTH,
            &mut error,
        );
        check_error(error);
    }

    let listCount = ob::ob_stream_profile_list_count(depthProfiles, &mut error);
    check_error(error);
    if listCount > 0 {
        if !color_profile.is_null() {
            // Select the profile with the same frame rate as color.
            let color_fps = ob::ob_video_stream_profile_fps(color_profile, &mut error);
            check_error(error);
            depth_profile = ob::ob_stream_profile_list_get_video_stream_profile(
                depthProfiles,
                ob::OB_WIDTH_ANY as c_int,
                ob::OB_HEIGHT_ANY as c_int,
                ob::OBFormat_OB_FORMAT_UNKNOWN,
                color_fps as c_int,
                &mut error,
            );
            check_error(error);
        }

        if (depth_profile.is_null()) {
            // If no matching profile is found, select the default profile.
            depth_profile = ob::ob_stream_profile_list_get_profile(
                depthProfiles,
                ob::OB_PROFILE_DEFAULT as c_int,
                &mut error,
            );
            check_error(error);
        }

        // enable stream
        ob::ob_config_enable_stream(ob_config, depth_profile, &mut error);
        check_error(error);

        // Turn on D2C alignment, which needs to be turned on when generating RGBD point clouds
        ob::ob_config_set_align_mode(ob_config, align_mode, &mut error);
        check_error(error);
    }

    // Get the device handle
    let ob_device: *mut ob::ob_device = ob::ob_pipeline_get_device(ob_pipeline, &mut error);
    check_error(error);

    // Start the pipeline with config
    ob::ob_pipeline_start_with_config(ob_pipeline, ob_config, &mut error);
    check_error(error);

    // Create a point cloud Filter object (device parameters will be obtained inside the Pipeline when the point cloud filter is created, so try to configure
    // the device before creating the filter)
    let point_cloud: *mut ob::ob_filter = ob::ob_create_pointcloud_filter(&mut error);
    check_error(error);

    // Obtain the current open-stream camera parameters from the pipeline and pass them to the point cloud filter
    let camera_param: ob::ob_camera_param =
        ob::ob_pipeline_get_camera_param(ob_pipeline, &mut error);
    check_error(error);
    ob::ob_pointcloud_filter_set_camera_param(point_cloud, camera_param, &mut error);
    check_error(error);

    let mut count = 0;
    let mut points_created = false;

    // Loop to get the frame and save the point cloud
    loop {
                count          = 0;
                points_created = false;
                // Limit up to 10 repetitions
                    // Waiting for one frame, the timeout is 100ms
                    let frameset: *mut ob::ob_frame = ob::ob_pipeline_wait_for_frameset(ob_pipeline, 100, &mut error);
                    check_error(error);
                    if !frameset.is_null() {
                        // get depth value scale
                        let depth_frame: *mut ob::ob_frame = ob::ob_frameset_depth_frame(frameset, &mut error);
                        check_error(error);
                        if(depth_frame.is_null()) {
                            continue;
                        }

                        // get depth value scale
                        let depth_value_scale: f32 = ob::ob_depth_frame_get_value_scale(depth_frame, &mut error);
                        check_error(error);

                        // delete depth frame
                        ob::ob_delete_frame(depth_frame, &mut error);
                        check_error(error);

                        // point position value multiply depth value scale to convert uint to millimeter (for some devices, the default depth value uint is not
                        // millimeter)
                        ob::ob_pointcloud_filter_set_position_data_scale(point_cloud, depth_value_scale, &mut error);
                        check_error(error);

                        ob::ob_pointcloud_filter_set_point_format(point_cloud, ob::OBFormat_OB_FORMAT_RGB_POINT, &mut error);
                        check_error(error);
                        let points_frame: *mut ob::ob_frame = ob::ob_filter_process(point_cloud, frameset, &mut error);
                        check_error(error);
                        if !points_frame.is_null() {
                            // PUSH
                            let points_size = ob::ob_frame_data_size(points_frame, &mut error) as usize / std::mem::size_of::<ob::OBColorPoint>();
                            check_error(error);

                            let points = ob::ob_frame_data(points_frame, &mut error) as *mut ob::OBColorPoint;
                            let points = std::slice::from_raw_parts_mut(points, points_size);
                            check_error(error);

                            for point in points.iter() {
                                let point = *point;
                                println!("point: x: {:?}", point);
                            }

                            ob::ob_delete_frame(points_frame, &mut error);
                            check_error(error);
                            points_created = true;
                        }
                        ob::ob_delete_frame(frameset, &mut error);  // Destroy frameSet to reclaim memory
                        check_error(error);
                        if(points_created) {
                            break;
                        }
                    }

        // Wait for up to 100ms for a frameset in blocking mode.
        let frameset = ob::ob_pipeline_wait_for_frameset(ob_pipeline, 100, &mut error);
        check_error(error);
        if !frameset.is_null() {
            ob::ob_delete_frame(frameset, &mut error); // Destroy frameSet to reclaim memory
            check_error(error);
            break;
        }
    }

    ob::ob_delete_filter(point_cloud, &mut error);
    check_error(error);

    // stop pipeline
    ob::ob_pipeline_stop(ob_pipeline, &mut error);
    check_error(error);

    // destroy pipeline
    ob::ob_delete_pipeline(ob_pipeline, &mut error);
    check_error(error);

    // destroy config
    ob::ob_delete_config(ob_config, &mut error);
    check_error(error);

    // destroy profile
    ob::ob_delete_stream_profile(depth_profile, &mut error);
    check_error(error);

    // destroy profile
    ob::ob_delete_stream_profile(color_profile, &mut error);
    check_error(error);

    // destroy profile list
    ob::ob_delete_stream_profile_list(color_profiles, &mut error);
    check_error(error);

    ob::ob_delete_stream_profile_list(depthProfiles, &mut error);
    check_error(error);
}
