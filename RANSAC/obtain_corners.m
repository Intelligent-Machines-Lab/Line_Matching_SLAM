function corner=obtain_corners(line_segment)
    segments = double(line_segment);
    N = size(segments,2)/2;
    corner1 = zeros(0,2);
    aux=1;
    for i = 1:N
        for j=1:N
            if i~=j
                [xi,yi] = polyxpoly(segments(1,2*i-1:2*i),segments(2,2*i-1:2*i),segments(1,2*j-1:2*j),segments(2,2*j-1:2*j));
                if ~isempty(xi) && ~isempty(yi)
                    corner1(aux,:)=[xi,yi];
                    aux=aux+1;
                end
            end
        end
    end
    [corner, ia, ic] = unique(corner1,'rows','stable');        
end