#
# based on https://stackoverflow.com/questions/28476117/
#                  easy-openstreetmap-tile-displaying-for-python
# and http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Python
# and http://wiki.openstreetmap.org/wiki/DE:Zoom_levels
#

import math
import urllib.request
import io
from PIL import Image
from pyproj import Proj, transform


def deg2num(lat_deg, lon_deg, zoom_level):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom_level
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
    return (xtile, ytile)

def num2deg(xtile, ytile, zoom_level):
    """
    http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
    This returns the NW-corner of the square. 
    Use the function with xtile+1 and/or ytile+1 to get the other corners. 
    With xtile+0.5 & ytile+0.5 it will return the center of the tile.
    """
    n = 2.0 ** zoom_level
    lon_deg = xtile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
    lat_deg = math.degrees(lat_rad)
    return (lat_deg, lon_deg)

def get_cluster_bbox(lat_deg, lon_deg, delta_lat,  delta_lon, zoom_level):
    xmin, ymax = deg2num(lat_deg, lon_deg, zoom_level)
    xmax, ymin = deg2num(lat_deg + delta_lat, lon_deg + delta_lon, zoom_level)

    bbox_ul = num2deg(xmin, ymin, zoom_level)
    bbox_ll = num2deg(xmin, ymax + 1, zoom_level)
    #print bbox_ul, bbox_ll

    bbox_ur = num2deg(xmax + 1, ymin, zoom_level)
    bbox_lr = num2deg(xmax + 1, ymax +1, zoom_level)
    #print bbox_ur, bbox_lr

    return [bbox_ll[1], bbox_ll[0], bbox_ur[1], bbox_ur[0]]

def get_image_cluster(lat_deg, lon_deg, delta_lat,  delta_lon, extent_width, extent_height):
    # define projections to calculate bounding boxes
    inProj = Proj(init='epsg:4326')
    outProj = Proj(init='epsg:3857')
    x0_desired, y0_desired = transform(inProj,outProj,lon_deg,lat_deg)
    x1_desired, y1_desired = transform(inProj,outProj,lon_deg+delta_lon,lat_deg+delta_lat)
    # calculate zoom level
    bbox_width = x1_desired-x0_desired
    bbox_height = y1_desired-y0_desired
    if extent_width/extent_height < bbox_width/bbox_height:
        m_per_pixel = bbox_width/extent_width
    else:
        m_per_pixel = bbox_height/extent_height
    equatorial_length_m = 40075016
    zoom_level = int(math.ceil(math.log(equatorial_length_m*math.cos(math.radians(lat_deg+delta_lat))/m_per_pixel,2)-8))
    print("Zoom level:", zoom_level)
    tile_bbox = get_cluster_bbox(lat_deg, lon_deg, delta_lat, delta_lon, zoom_level)
    x0_tiles, y0_tiles = transform(inProj,outProj,tile_bbox[0],tile_bbox[1])
    x1_tiles, y1_tiles = transform(inProj,outProj,tile_bbox[2],tile_bbox[3])
    # download and cluster tiles
    smurl = r"http://a.tile.openstreetmap.org/{0}/{1}/{2}.png"
    xmin, ymax = deg2num(lat_deg, lon_deg, zoom_level)
    xmax, ymin = deg2num(lat_deg + delta_lat, lon_deg + delta_lon, zoom_level)
    cluster = Image.new('RGB',((xmax-xmin+1)*256-1,(ymax-ymin+1)*256-1) )
    num_tiles = (xmax-xmin+1)*(ymax-ymin+1)
    if num_tiles > 50:
        print("Error: more than 50 tiles (%s), canceling download" % num_tiles)
        return
    else:
        print("Download %s tiles" % num_tiles)
        for xtile in range(xmin, xmax+1):
            for ytile in range(ymin,  ymax+1):
                try:
                  img_url = smurl.format(zoom_level, xtile, ytile)
                  print("Opening: " + img_url)
                  req = urllib.request.Request(img_url, headers={'User-Agent': 'Custom user agent'})
                  img_bytes = urllib.request.urlopen(req)
                  tile = Image.open(img_bytes)
                  cluster.paste(tile, box=((xtile-xmin)*255 ,  (ytile-ymin)*255))
                except:
                    print("Couldn't download image")
                    return

    # cropping
    scaling_x = cluster.size[0]/(x1_tiles-x0_tiles)
    scaling_y = cluster.size[1]/(y1_tiles-y0_tiles)
    crop_left = int(scaling_x*(x0_desired-x0_tiles))
    crop_bottom = int(scaling_y*(y0_desired-y0_tiles))
    crop_right = int(scaling_x*(x1_tiles-x1_desired))
    crop_top = int(scaling_y*(y1_tiles-y1_desired))
    cropped = cluster.crop(box=(crop_left,crop_top,cluster.size[0]-crop_right,cluster.size[1]-crop_bottom))

    return cropped
