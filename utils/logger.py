import csv
import os


class CSVLogger:
    def __init__(self, output_path):
        self.output_path = output_path
        os.makedirs(os.path.dirname(output_path), exist_ok=True)

        self.fieldnames = [
            "time",
            "x",
            "y",
            "yaw",
            "speed",
            "steering",
            "reference_x",
            "reference_y",
            "cross_track_error",
            "heading_error",
        ]

        self.file = open(output_path, mode="w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames)
        self.writer.writeheader()

    def log(self, row):
        self.writer.writerow(row)

    def close(self):
        self.file.close()