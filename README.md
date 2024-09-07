# ERC_Drone

## MATLAB Code to Preprocess AruCo Marker

![image](https://github.com/user-attachments/assets/9b0c12d7-2c55-4dcc-9a5e-21a1abe1ab37)
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
