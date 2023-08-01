% Face detect and track. Computer Vision System Toolbox is required.
% modified by S.Grys 23.05.2023
% --------------------------
% set path to ...\for_training first
% run this script to capture a fresh face
% (to do it please close the camera preview window by Alt+F4)
% train the classifier
% folder=pwd;
  % [images,H,W,M,m,U,omega]=trainingEF(folder);
% ignore some errors like 'Unrecognized function or variable 'images'.'
% look at variables in Workspace 
% set path to ...\for_testing
% run this script again to capture a face for comparison
% (to do it please close the camera preview window by Alt+F4)

if exist('captured.pgm')
delete('captured.pgm')
end
   
% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();

% Create the point tracker object.
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Create the webcam object.
cam = webcam();

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]]);

runLoop = true;
release(videoPlayer);
while runLoop
    % Get the next frame.
    videoFrame = snapshot(cam);
    videoFrameGray = rgb2gray(videoFrame);
        bbox=[];
            bbox = faceDetector.step(videoFrameGray);
        if ~isempty(bbox)
            % Find corner points inside the detected region.
            points = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox(1, :));

            % Re-initialize the point tracker.
            xyPoints = points.Location;
            release(pointTracker);
            initialize(pointTracker, xyPoints, videoFrameGray);

            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints = bbox2points(bbox(1,:));

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);

            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3, 'Color'  ,'Red');

            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints, '+', 'Color', 'white');
bbox_exp=bbox(1,1:2);       
bbox_exp(1)=bbox_exp(1)-50;
bbox_exp(2)=bbox_exp(2)-50;
bbox_exp(3:4)=299;
bbox_exp
videoFrameGray_Crop=imcrop(videoFrameGray,bbox_exp);
        end
    % Display the annotated video frame using the video player object.
         step(videoPlayer, videoFrame);
         
    % Check whether the video player window has been closed.
      runLoop = isOpen(videoPlayer);
    if size(videoFrameGray_Crop)~=[300 300] runLoop=true; end
    %let's grab a new photo
    if ((runLoop==false) && ~isempty(bbox)) 
       imwrite(videoFrameGray_Crop,'captured.pgm');
       testingEF('captured.pgm',images,H,W,M,m,U,omega);
       disp('Pattern saved to file successfully');
       pause;
       runLoop=true;
       release(videoPlayer); 
    end
end
% Clean up.
clear cam;
release(videoPlayer);
release(pointTracker);
release(faceDetector);