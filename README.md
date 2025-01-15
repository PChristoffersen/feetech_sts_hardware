# feetech_sts_hardware

A [`ros2_control`](https://github.com/ros-controls/ros2_control) implementation for [Feetech](https://www.feetechrc.com/) STS servos.

Only tested with the STS3215 servo, but it may work with other servos in the series.

## Servo configuration parameters

```xml
<ros2_control name="${name}" type="system">
    <hardware>
        <plugin>feetech_sts_hardware/FeetechSTSHardware</plugin>
        <param name="port_name">/dev/ttyACM0</param>
        <param name="baud_rate">1000000</param>
        <!-- <param name="default_zero_offset">2027</param> -->
    </hardware>

    <joint name="${prefix}0_coxa_joint">
        <param name="id">1</param>
        <!-- <param name="zero_offset">2027</param> -->
        <command_interface name="position"/>
        <command_interface name="velocity"/>
        <command_interface name="acceleration"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
    </joint>
</ros2_control>
```


## FTServo_Linux library

This plugin includes the [Feetech servo library](https://gitee.com/ftservo/FTServo_Linux) in `src/SCServo`. See the `LICENSE` file in that directory for license information.

## License

All code except the file in `src/SCServo` are licensed under the BSD 3-Clause License.

