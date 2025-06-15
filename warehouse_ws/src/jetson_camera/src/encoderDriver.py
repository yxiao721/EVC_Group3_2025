#!/usr/bin/env python2

from enum import IntEnum
import Jetson.GPIO as GPIO
import rospy
import time


class WheelDirection(IntEnum):
    FORWARD = 1
    REVERSE = -1


class WheelEncoderDriver:
    """Class handling communication with a wheel encoder.

    An instance of this class reads data off of a wheel encoder and calls a callback function
    with the new cumulative tick number as the sole argument.
    The callback is called only when the encoder fires, thus there is no constant frequency.

    Args:
        gpio_pin (:obj:`int`): ID of the pin the encoder is connected to.
    """

    def __init__(self, gpio_pin):
        # Validate gpio_pin
        if not 1 <= gpio_pin <= 40:
            raise ValueError("The pin number must be within the range [1, 40].")

        self._gpio_pin = gpio_pin
        GPIO.setmode(GPIO.BCM)
        #except ValueError as e:
        #    if "A different mode has already been set!" not in str(e):
        #        rospy.loginfo("BCM error")
        #        raise
        GPIO.setup(gpio_pin, GPIO.IN)
        GPIO.add_event_detect(gpio_pin, GPIO.RISING, callback=self._cb)

        self._ticks = 0
        self._direction = WheelDirection.FORWARD

        # For timing between ticks
        self._last_tick_time = None
        self._prev_tick_time = None

    def get_direction(self):
        return self._direction

    def set_direction(self, direction):
        self._direction = direction

    def _cb(self, _):
        now = time.time()
        self._prev_tick_time = self._last_tick_time
        self._last_tick_time = now
        self._ticks += self._direction.value

    def get_dt(self):
        """Return time difference between the last two ticks in seconds, or None if too long."""
        if self._prev_tick_time is None or self._last_tick_time is None:
            return None  # Not enough data yet

        dt = self._last_tick_time - self._prev_tick_time
        if dt > 2.0:
            return None
        return dt

    def shutdown(self):
        GPIO.remove_event_detect(self._gpio_pin)
