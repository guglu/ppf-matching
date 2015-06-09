function [sorted, sortIdx] = sortPoses(poses)
%sort cell of the poses according to the number of votes
    
    %extract number of votes
    numVotes = cellfun(@(x)x.numVotes, poses);

    [~,sortIdx] = sort(numVotes,'descend');
    
    sorted = poses(sortIdx);

end