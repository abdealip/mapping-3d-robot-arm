#!/usr/bin/python3

import os
import cv2
import pandas as pd
from argparse import ArgumentParser

class VideoStitcher:
    def __init__(self, video_dir, outfilename, framerate):
        image = cv2.imread(os.path.join(video_dir, "frame0000.png"))
        self.video_dir = video_dir
        self.video = cv2.VideoWriter(outfilename, 0, framerate, (image.shape[1], image.shape[0]))
        self.index = pd.read_csv(os.path.join(video_dir, "img_index.csv"))

    def make_video(self):
        # last_timestamp = None
        # last_img = None
        # frames_written = 0
        n_rows = len(self.index)
        for i in range(n_rows):
            row = self.index.iloc[i]
            # timestamp = row["timestamp"]
            img = cv2.imread(os.path.join(self.video_dir, row["imgfile"]))
            # if last_timestamp == None:
            #     self.video.write(img)
            #     frames_written += 1
            # else:
            #     while timestamp > last_timestamp + self.frame_period:
            #         self.video.write(last_img)
            #         frames_written += 1
            self.video.write(img)
            #     frames_written += 1
            print(f"Iteration {i+1}/{n_rows}")
            # last_timestamp = timestamp
            # last_img = img
        self.video.release()

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-f", "--framerate", type=float, required=True, help="Video Time Between Frames (Seconds)")
    parser.add_argument("-d", "--image-dir", required=True, help="Directory where image data is stored")
    parser.add_argument("-o", "--video-file", type=str, required=True, help="Filename to Save Output To")
    args = vars(parser.parse_args())

    video_stitcher = VideoStitcher(args["image_dir"], args["video_file"], args["framerate"])
    video_stitcher.make_video()

