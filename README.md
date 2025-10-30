# IGVC
Self driving car yo

Bird's eye overview of the project:
1. Capture data from image and depth
2. Clean data
3. Feed to two NNs
   a. YOLO returns array of "boxes" : { enum type, float x, float y, float h, float w }
   b. 2nd returns lanes as **POSSIBLY** a mathematical function
4. Check depth data in the YOLO "boxes" for the closest point. Assign each box that depth.
5. Assume max speed. Make decisions based on the captured data

Tasks
1. Implement a ros node to publish "boxes" from image data.
2. Implement a ros node to publish lane data from images.
3. Implement a ros node to subscribe to "boxes" and assign a depth
4. idk
