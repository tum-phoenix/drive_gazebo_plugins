import pandas as pd
import sys
import cv2

# change this to the output folder specified in the launch file
source_folder = '/hdd/ground_truth/'
df = pd.read_csv(source_folder + 'ground_truth.csv')
for index, row in df.iterrows():
    img = cv2.imread(source_folder + row['Filename'], cv2.IMREAD_UNCHANGED)
    cv2.rectangle(img, (int(row['Roi.X1']), int(row['Roi.Y1'])), (int(row['Roi.X2']), int(row['Roi.Y2'])), (0, 255, 0))
    cv2.imshow('image',img)
    cv2.waitKey(10000)

cv2.destroyAllWindows()


