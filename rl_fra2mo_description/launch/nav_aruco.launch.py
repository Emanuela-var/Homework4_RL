
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


 

def generate_launch_description():
    

    other_launch_file_navigation = PathJoinSubstitution(
        [FindPackageShare("rl_fra2mo_description"), "launch", "fra2mo_explore.launch.py"]
    )

    
    include_other_launch_navigation= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file_navigation)
    )

    other_launch_file_aruco= PathJoinSubstitution(
        [FindPackageShare("aruco_ros"), "launch", "aruco_cam.launch.py"]
    )

    
    include_other_launch_auco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file_aruco)
    )

    
    return LaunchDescription([
        include_other_launch_navigation, 
        include_other_launch_auco

    ])
