function do_polygons_intersect(a, b)
    polygons = [a, b];
    minA, maxA, projected, i, i1, j, minB, maxB = None, None, None, None, None, None, None, None
    for i in range(len(polygons))
        polygon = polygons[i];
        for i1 in range(len(polygon))
            i2 = (i1 + 1) % len(polygon);
            p1 = polygon[i1];
            p2 = polygon[i2];

            normal = { 'x': p2[1] - p1[1], 'y': p1[0] - p2[0] };

            minA, maxA = None, None
       
            for j in range(len(a)):
                projected = normal['x'] * a[j][0] + normal['y'] * a[j][1];
                if (minA is None) or (projected < minA):
                    minA = projected
                end
                if (maxA is None) or (projected > maxA):
                    maxA = projected
                end
            end
            minB, maxB = None, None
            for j in range(len(b)):
                projected = normal['x'] * b[j][0] + normal['y'] * b[j][1]
                if (minB is None) or (projected < minB):
                    minB = projected
                end
                if (maxB is None) or (projected > maxB):
                    maxB = projected
                end
            end
            if (maxA < minB) or (maxB < minA):
                print("polygons don't intersect!")
                return False;
            end
        end
    end
    return True
end