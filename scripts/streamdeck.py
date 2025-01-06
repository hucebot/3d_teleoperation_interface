#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import threading
import time

from PIL import Image, ImageDraw, ImageFont, ImageColor
from StreamDeck.DeviceManager import DeviceManager
from StreamDeck.ImageHelpers import PILHelper
from std_msgs.msg import Bool


class StreamDeckController(Node):
    def __init__(self):
        super().__init__('stream_deck_controller')
        self.exit_loop = False

        self.update_point_cloud = False
        self.update_point_cloud_publisher = self.create_publisher(Bool, '/update_point_cloud', 10)
        self.rate = 0.1

        self.key_width = 100
        self.key_height = 100

        self.background_color = "#bababa"
        self.color_text = "#000000"
        self.background_color_active = "#27a007"
        self.background_color_inactive = "#bf1616"

        try:
            self.stream_deck = DeviceManager().enumerate()[0]
            self.stream_deck.open()
            self.stream_deck.reset()
            self.stream_deck.set_brightness(100)
            self.get_logger().info("Found Stream Deck device.")
        except IndexError:
            self.get_logger().error("No Stream Deck devices were found")
            self.exit_loop = True

        self.background_image = Image.new("RGB", (self.key_width, self.key_height), color=ImageColor.getrgb("#000000"))
        
        self.column, self.row = self.stream_deck.key_layout()

        self.initialize_buttons()
        self.stream_deck.set_key_callback(self.on_key_change)
        
        self.main_loop()

    def main_loop(self):
        while not self.exit_loop:
            pass

        self.stream_deck.set_brightness(0)
        self.stream_deck.close()

    def on_key_change(self, deck, key, state):
        if state:
            if key == self.update_point_cloud_button_position:
                self.update_point_cloud = True
                self.update_point_cloud_publisher.publish(Bool(data=True))
                self.create_button(self.update_point_cloud_button_position, "Update Point Cloud", self.background_color_active)

        else:
            self.create_button(self.update_point_cloud_button_position, "Update Point Cloud", self.background_color_inactive)

    def create_button(self, position, label, background_color):
        image = self.background_image.copy()
        label = label.upper()
        x_pos = 0
        y_pos = 40

        if " " not in label:
            x_pos = 50 - (len(label) * 5)

        else:
            list_words = label.split(" ")
            label = ""
            for word in list_words:
                len_word = len(word)
                if len_word > 7:
                    label += " "*4 + word + "\n"

                elif len_word > 5:
                    label += " "*7 + word + "\n"

                else:
                    label += " "*10 + word + "\n"
            

        draw = ImageDraw.Draw(image)
        draw.rectangle([(0, 0), (self.key_width, self.key_height)], fill=ImageColor.getrgb(background_color))

        if "EMERGENCY" in label:
            self.color_text = "#ffffff"
        else:
            self.color_text = "#000000"

        draw.text((x_pos, y_pos), label, fill=ImageColor.getrgb(self.color_text))
        
        key_image = PILHelper.to_native_format(self.stream_deck, image)
        
        self.stream_deck.set_key_image(position, key_image)
        return position


    def initialize_buttons(self):
        #### Update Point Cloud Button
        self.update_point_cloud_button_position = (0, 0)
        self.update_point_cloud_button_position = self.update_point_cloud_button_position[0] * self.row + self.update_point_cloud_button_position[1]
        self.update_point_cloud_button = self.create_button(self.update_point_cloud_button_position, "Update Point Cloud", self.background_color)


if __name__ == '__main__':
    rclpy.init()
    stream_deck_controller = StreamDeckController()
    rclpy.spin(stream_deck_controller)
    rclpy.shutdown()
