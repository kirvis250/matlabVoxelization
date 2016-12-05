function [inSegment, IntersectionX, IntersectionY, IntersectionZ]=lineIntersectsTriangle(Line1,Line2, V1,V2,V3)
% Line1 - start point of segment
% Line2 - end point of segment
% V1, V2, V3 - vertices of triangle (coordinates)
inSegment=false; 
LineX1=Line1(1); LineY1=Line1(2); LineZ1=Line1(3); 
LineX2=Line2(1); LineY2=Line2(2); LineZ2=Line2(3);


    % LinePlaneIntersectionByPoints
    [IsIntersection, IsOverlap, IntersectionX, IntersectionY, IntersectionZ] = ...
            LinePlaneIntersectionByPoints(LineX1, LineY1, LineZ1, LineX2, LineY2, LineZ2, V1(1), V1(2), V1(3), V2(1), V2(2), V2(3), V3(1), V3(2), V3(3));
    if(IsIntersection),
        % IsCoplanar3DPointInsideConvexPolygon
        inside = IsCoplanar3DPointInsideConvexPolygon(IntersectionX, IntersectionY, IntersectionZ, [V1(1),V2(1),V3(1)], [V1(2),V2(2),V3(2)], [V1(3),V2(3),V3(3)]);
        if(inside),
%             % point in line segment
            if  (dot([IntersectionX, IntersectionY, IntersectionZ]-[LineX1, LineY1, LineZ1],[LineX2, LineY2, LineZ2]-[LineX1, LineY1, LineZ1]) >= 0) & (dot([IntersectionX, IntersectionY, IntersectionZ]-[LineX2, LineY2, LineZ2],[LineX2, LineY2, LineZ2]-[LineX1, LineY1, LineZ1]) <= 0)
%                %  --- POINT ---- 
                inSegment=true;
            end
        end
        
    end
end



function [IsInside] = IsCoplanar3DPointInsideConvexPolygon(PointX, PointY, PointZ, PolygonX, PolygonY, PolygonZ)
    % IsCoplanar3DPointInsideConvexPolygon - is the 3D point inside polygon
    % assumed to be convex? Point is assumed to be in the same
    % plane.
    if (all(PolygonX == PolygonX(1)))
        [IsInside] = Is2DPointInsideConvexPolygon(PointY, PointZ, PolygonY, PolygonZ);
    else
        if (all(PolygonY == PolygonY(1)))
            [IsInside] = Is2DPointInsideConvexPolygon(PointX, PointZ, PolygonX, PolygonZ);
        else
            [IsInside] = Is2DPointInsideConvexPolygon(PointX, PointY, PolygonX, PolygonY);
        end
    end
end

        
  
function [IsInside] = Is2DPointInsideConvexPolygon(PointX, PointY, PolygonX, PolygonY)
    % Is2DPointInsideConvexPolygon - is the 2D point inside polygon
    % assumed to be convex?
    if (length(PolygonX) ~= length(PolygonY))
        error('Polygon must have an equal number of x and y coordinates.');
    end
    IsInside = true;
    if (PointX < min(PolygonX)) || (PointY < min(PolygonY)) || (PointX > max(PolygonX)) || (PointY > max(PolygonY))
        IsInside = false;
    else
        [~, IsOnLeftSide, IsOnLine, ~] = Is2DPointOnTheLeftOfLine(PointX, PointY, PolygonX(end), PolygonY(end), PolygonX(1), PolygonY(1));
        IsOnLeft = IsOnLeftSide;
        if (IsOnLine)
            IsInside = true;
        else
            for VertexNr = 1:(length(PolygonX)-1)
                [~, IsOnLeftSide, IsOnLine, ~] = Is2DPointOnTheLeftOfLine(PointX, PointY, PolygonX(VertexNr), PolygonY(VertexNr), PolygonX(VertexNr+1), PolygonY(VertexNr+1));
                if (IsOnLine)
                    IsInside = true;
                    break;
                end
                if (IsOnLeft ~= IsOnLeftSide)
                    IsInside = false;
                    break;
                end
            end
        end
    end
end

function [NumLeft, IsOnLeftSide, IsOnLine, IsOnRightSide] = Is2DPointOnTheLeftOfLine(PointX, PointY, LineX1, LineY1, LineX2, LineY2)
    %http://geomalgorithms.com/a03-_inclusion.html
    NumLeft = (LineX1 - PointX) * (LineY2 - PointY) - (LineX2 - PointX) * (LineY1 - PointY);
    
    if(abs(NumLeft) < 1e-16) NumLeft  = 0; end;
    
    IsOnLeftSide = NumLeft > 0;
    IsOnLine = NumLeft == 0;
    IsOnRightSide = NumLeft < 0;
end
  
  
function [IsIntersection, IsOverlap, IntersectionX, IntersectionY, IntersectionZ] = ...
        LinePlaneIntersectionByPoints(LineX1, LineY1, LineZ1, LineX2, LineY2, LineZ2, PlaneX1, PlaneY1, PlaneZ1, PlaneX2, PlaneY2, PlaneZ2, PlaneX3, PlaneY3, PlaneZ3)
    % LinePlaneIntersectionByPoints - finds out if the line given
    % by two points intersects the plane given by two points.
    % Using http://mathworld.wolfram.com/Line-PlaneIntersection.html
    IntersectionTSkait = det([ 1, 1, 1, 1;
        PlaneX1, PlaneX2, PlaneX3, LineX1;
        PlaneY1, PlaneY2, PlaneY3, LineY1;
        PlaneZ1, PlaneZ2, PlaneZ3, LineZ1]);
    IntersectionTVard = det([ 1, 1, 1, 0;
        PlaneX1, PlaneX2, PlaneX3, LineX2-LineX1;
        PlaneY1, PlaneY2, PlaneY3, LineY2-LineY1;
        PlaneZ1, PlaneZ2, PlaneZ3, LineZ2-LineZ1]);
    if (IntersectionTVard ~= 0)
        IsIntersection = true;
        IsOverlap = false;
        IntersectionT = -IntersectionTSkait / IntersectionTVard;
        IntersectionX = LineX1 + (LineX2 - LineX1) * IntersectionT;
        IntersectionY = LineY1 + (LineY2 - LineY1) * IntersectionT;
        IntersectionZ = LineZ1 + (LineZ2 - LineZ1) * IntersectionT;
    else
        IsIntersection = false;
        IntersectionX = 0;
        IntersectionY = 0;
        IntersectionZ = 0;
        if (IntersectionTSkait ~= 0)
            IsOverlap = false;
        else
            IsOverlap = true;
        end
    end
end