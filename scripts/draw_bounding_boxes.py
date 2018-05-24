import pandas as pd
import sys
import cv2
import argparse, os

parser = argparse.ArgumentParser(description="Visualize the output of sign_label_plugin")
parser.add_argument("csv_file", nargs="?", type=argparse.FileType("r"), default=sys.stdin)
args = parser.parse_args()

# change this to the output folder specified in the launch file
source_folder = os.path.dirname(args.csv_file.name)
df = pd.read_csv(args.csv_file.name)
for index, row in df.iterrows():
    img = cv2.imread(source_folder + '/' + row['Filename'], cv2.IMREAD_UNCHANGED)
    cv2.namedWindow('Label in image', flags=cv2.WINDOW_NORMAL)
    cv2.rectangle(img, (int(row['Roi.X1']), int(row['Roi.Y1'])), (int(row['Roi.X2']), int(row['Roi.Y2'])), (0, 255, 0))
    cv2.imshow('Label in image', img)
    cv2.waitKey(10000)

cv2.destroyAllWindows()


