from pyproj import Proj, transform
from mpl_toolkits.basemap import Basemap

def wgs84_to_web_mercator(lat, lon, height=0):
    # WGS 84 (GPS)
    in_proj = Proj(init='epsg:4326')
    # Pseudeo Mercator (OSM)
    out_proj = Proj(init='epsg:3857')
    return transform(in_proj, out_proj, lat, lon, height)

def get_utm_lon_lat_0(lon, lat):
    # Calculate reference UTM grid zone
    if lat>=72:
        lat_0 = 72
        if 0<=lon and lon<= 9:
            lon_0 = 0
        elif 9<=lon and lon<= 21:
            lon_0 = 9
        elif 21<=lon and lon<= 33:
            lon_0 = 21
        elif 33<=lon and lon<= 42:
            lon_0 = 33
        else:
            lon_0 = int(lon/6)*6

    elif 56<=lat and lat<= 64:
        lat_0 = 56
        if 3<=lon and lon<=12:
            lon_0 = 3
        else:
            lon_0 = int(lon/6)*6

    else:
        lat_0 = int(lat/8)*8
        lon_0 = int(lon/6)*6

    return (lon_0, lat_0)