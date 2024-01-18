rosbag record -O rgb_cam_cali_record.bag camera/color/image_raw camera/color/camera_info
python3 bag_to_images.py ./rgb_cam_cali_record.bag ./output camera/color/image_raw
rosrun camera_models Calibrations -w 8 -h 6 -s 25 -i ./output --camera-model pinhole --prefix frame

