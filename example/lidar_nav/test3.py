from pyhull.convex_hull import ConvexHull

pts = [[-0.5, -0.5], [-0.5, 0.5], [0.5, -0.5], [0.5, 0.5], [0,0]]
hull = ConvexHull(pts)


print(hull.vertices)