imgIdx = 45;
testFeat = featAll;

for imgIdx = 1:200

    im = imread(fullfile(pascalImagesDir, [dataStruct.voc_image_id{imgIdx} '.jpg']));
    testFeat = featConv(imgIdx, :);
    bbox = dataStruct.bbox(imgIdx,:);
    
    switch params.predMethod
        
        case 'maxLocation'
            % disp('maxLocation method');
            [kpCoords,scores] = maxLocationPredict(testFeat, bbox, params.heatMapDims);
            kpCoords = kpCoords(1:2, 1:14);
            [b, bi] = sort(scores, 'descend');
            [~, ind] = find(scores >= 0.99);
            kpCoords = kpCoords(1:2, ind);
            
        case 'maxLocationCandidates'
            % disp('maxLocationCandidates method');
            [kpCoords,scores] = maxLocationCandidates(testFeat, bbox, params.heatMapDims);
            scoresTemp = [];
            for idx = 1:14
                scoresTemp = [scoresTemp; scores{1,idx}];
            end
            scores = scoresTemp;
            % kpCoords = kpCoords(1:2, 1:14);
            % kpCoords = kpCoords{1:14};
            kpCoordsTemp = [];
            for idx = 1:length(kpCoords)
                kpCoordsTemp = [kpCoordsTemp, kpCoords{idx}];
            end
            kpCoords = kpCoordsTemp;
    end
    
    bbox2(1) = bbox(1); bbox2(2) = bbox(2); bbox2(3) = bbox(3)-bbox(1); bbox2(4) = bbox(4)-bbox(2);
    imshow(im);
    hold on
    scatter(kpCoords(1,:),kpCoords(2,:),50,'r','filled')
    % scatter(kps(:,1),kps(:,2),50,'b','filled')
    rectangle('Position', bbox2, 'LineWidth', 3, 'EdgeColor', 'g');
    hold off
    
    pause;

end