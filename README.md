# ERC_Drone

## MATLAB Code to Preprocess AruCo Marker

```
function processImage(image_path)
    image = imread(image_path);

    gray_image = rgb2gray(image);

    processed_image = imadjust(wiener2(gray_image, [5 5]), [], [], 0.1);

    binary_image = imbinarize(processed_image);

    % [ids, corners] = detectArucoMarkers(binary_image);

    % if ~isempty(ids)
        % marker_id = ids(1);
        % filename = sprintf('marker_%d.jpg', marker_id);
        imwrite(binary_image, 'prprocessed_aruco.jpg');
        % disp(['Marker ID: ', num2str(marker_id)]);
    % else
        % disp('No markers detected');
    % end
end

image_path = uigetfile('aruco.jpg');

processImage(image_path);
```
