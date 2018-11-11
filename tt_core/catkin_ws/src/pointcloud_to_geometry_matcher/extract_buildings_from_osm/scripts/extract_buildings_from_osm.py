#!/usr/bin/env python
import osmnx as ox
import matplotlib.pyplot as plt

LAT_LO = 43.0776
LAT_HI = 43.0880
LON_LO = -77.6873
LON_HI = -77.6640


# geo_bbox = ox.Polygon([(LAT_LO, LON_LO),
#                        (LAT_LO, LON_HI),
#                        (LAT_HI, LON_HI),
#                        (LAT_HI, LON_LO)])

# x, y = geo_bbox.exterior.xy
# plt.plot(x, y, 'o')
# plt.show()

# gdf = ox.buildings_from_polygon(geo_bbox)
gdf = ox.create_buildings_gdf(north=LAT_HI, south=LAT_LO,
                              east=LON_HI, west=LON_LO)

gdf_proj = ox.project_gdf(gdf)
#ox.plot_buildings(gdf_proj)

ox.save_gdf_shapefile(gdf, filename='rit_buildings', folder='.')

# pcl::PointCloud<pcl::PointXYZI>
# STL, VTK, OBJ, PLY
