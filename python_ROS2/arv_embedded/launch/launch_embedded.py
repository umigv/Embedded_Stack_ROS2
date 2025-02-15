import launch
import launch_ros.actions
import launch.actions

def generate_launch_description():
    # Declare launch argument for LED control
    use_LED_arg = launch.substitutions.LaunchConfiguration('use_LED')

	# USB permission action (only executed if use_LED is true)
    usb_permission_action = launch.actions.ExecuteProcess(
        cmd=["bash", "-c", "'sudo chmod 666 /dev/ttyACM0'"],
        shell=True,
        output='screen',
        condition=launch.conditions.IfCondition(use_LED_arg)
    )
    # Define the LED node (only launched if use_LED is true)
    led_node = launch_ros.actions.Node(
        package='arv_embedded', 
        executable='LED_subscriber',
        output='screen',
        condition=launch.conditions.IfCondition(use_LED_arg)
    )

    # These nodes always launch
    dual_odrive_controller_node = launch_ros.actions.Node(
        package='arv_embedded', 
        executable='dual_odrive_controller',
        output='screen'
    )

    enc_odom_publisher_node = launch_ros.actions.Node(
        package='arv_embedded', 
        executable='enc_odom_publisher',
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('use_LED', default_value='false', description="Enable LED node and set USB permissions"),
        usb_permission_action,
        dual_odrive_controller_node,
        enc_odom_publisher_node,
        led_node
    ])

