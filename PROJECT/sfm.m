imageDir = 'C:\Users\M Osama Nusrat\OneDrive - National University of Sciences & Technology\Desktop\best matching image';
%imageDir = 'C:\Users\M Osama Nusrat\OneDrive - National University of Sciences & Technology\Desktop\iiii';
imageType = fullfile(imageDir, '*jpeg');
% Struct imSet with fields: (name, date, bytes, isdir, datenum)
imStruct = dir(imageType);
% Initialize figure.
figure
% store image file names.
imgs = {imStruct.name};
I = undistortImage(images{1}, cameraParams); 
% Initialize empty array for storing images.
images = cell(1, numel(imgs));
filenames = cell(1, numel(imgs));
for i = 1:numel(imgs)
    % Get full path of image.
    full_path = fullfile(imageDir, imgs{i});
    % Add string to filenames.
    filenames{i} = full_path;
    % Read grayscale image .
    I = imread(full_path);
    % Store in array.
    images{i} =rgb2gray(I);
end

% Display images.
montage(filenames)

title('Input Image Sequence');

%--------------------------------------------------------------------------

%data = load(fullfile(imageDir, 'cameraParams.mat'));
%cameraParams = data.cameraParams;

% Get intrinsic parameters of the camera
% cameraParams = cameraParams.IntrinsicMatrix;

% Undistort the first image.
imshow(images{1})
I = undistortImage(images{1}, cameraParams); 
imshow(I)
size(I)

%--------------------------------------------------------------------------

% Detect features. Increasing 'NumOctaves' helps detect large-scale
% features in high-resolution images. Use an ROI to eliminate spurious
% features around the edges of the image.
border = 50;
roi = [border, border, size(I, 2)- 2*border, size(I, 1)- 2*border];
prevPoints = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);

% Extract features. Using 'Upright' features improves matching, as long as
% the camera motion involves little or no in-plane rotation.
prevFeatures = extractFeatures(I, prevPoints, 'Upright', true);

% Create an empty imageviewset object to manage the data associated with each
% view.
vSet = viewSet;

% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', ...
    eye(3), 'Location', [0 0 0]);

prev_I = I;
%--------------------------------------------------------------------------

for i = 2:numel(images)
    % Undistort the current image.
    I = undistortImage(images{i}, cameraParams);
    
    % Detect, extract and match features.
    currPoints   = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);
    currFeatures = extractFeatures(I, currPoints, 'Upright', true);    
    indexPairs   = matchFeatures(prevFeatures, currFeatures, ...
        'MaxRatio', .95, 'Unique',  true);
    
    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
    
    figure; ax = axes;
    showMatchedFeatures(prev_I,I,matchedPoints1,matchedPoints2,'montage','Parent',ax);
    title(ax, 'Candidate point matches');
    legend(ax, 'Matched points 1','Matched points 2');
    hold('on')
    
    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(...
        matchedPoints1, matchedPoints2, cameraParams);
    %[E, inlierIdx] = estimateEssentialMatrix(matchedPoints1, matchedPoints2,...
    %cameraParams, 'Confidence', 50, 'MaxDistance', 5);
    % Add the current view to the view set.
    vSet = addView(vSet, i, 'Points', currPoints);
    
    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, 'Matches', indexPairs(inlierIdx,:));
    
    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1);
    prevOrientation = prevPose.Orientation{1}; 
    prevLocation = prevPose.Location{1};
        
    % Compute the current camera pose in the global coordinate system 
    % relative to the first view.
    orientation = prevOrientation * relativeOrient;
    location = prevLocation + relativeLoc * prevOrientation;
    vSet = updateView(vSet, i, 'Orientation', orientation, ...
        'Location', location);

    % Find point tracks across all views.
    tracks = findTracks(vSet);

    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);
    
    % Refine the 3-D world points and camera poses.
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
        'PointsUndistorted', true);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    prevPoints   = currPoints;  
    prev_I = I;
end
