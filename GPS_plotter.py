import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap
import numpy as np

file_names = ['GPS_log_GT_U7_2208.txt', 
              'GPS_log_GT_U7_2212.txt',
              'GPS_log_GY_GPS6MV2.txt']
              # 'GPS_log_Austin_Indoor.txt']

min_lat = np.inf
max_lat = -np.inf
min_lon = np.inf
max_lon = -np.inf

data = []
for file_name in file_names:
    try:
        with open('EE 475\\' + file_name, 'r') as file:
            # Initialize empty lists to store latitude and longitude values
            latitudes = []
            longitudes = []
            # Read each line from the file
            for line in file:
                # Split the line into components
                components = line.strip().split(',')
                # Extract latitude and longitude values
                latitude = int(components[0][:2]) + float(components[0][2:])/60
                longitude = int(components[2][:3]) + float(components[2][3:])/60
                # Convert latitude and longitude to negative values if 'S' or 'W' is present
                if components[1] == 'S':
                    latitude = -latitude
                if components[3] == 'W':
                    longitude = -longitude
                # Append values to the lists
                latitudes.append(latitude)
                longitudes.append(longitude)

                if latitude < min_lat: min_lat = latitude
                if latitude > max_lat: max_lat = latitude
                if longitude < min_lon: min_lon = longitude
                if longitude > max_lon: max_lon = longitude

            data.append((np.array(latitudes), np.array(longitudes), file_name))
            
    except FileNotFoundError:
        print(f"Error: File '{file_name}' not found.")

# Create a basemap centered at the mean coordinates of the data
map_plot = Basemap(projection='mill', llcrnrlat=min_lat, urcrnrlat=max_lat,
                   llcrnrlon=min_lon, urcrnrlon=max_lon, resolution='c')

for i in range(10):
    plt.axvline(x=10*i, c='black', linewidth=1)
    plt.axhline(y=10*i, c='black', linewidth=1)

for lat, lon, file_name in data:  
    lat_mu = lat.mean()
    lat_std = lat.std()
    lon_mu = lon.mean()
    lon_std = lon.std()

    print(lat_std, lon_std)

    map_plot.drawgreatcircle(lon_mu-lon_std, lat_mu-lat_std, lon_mu+lon_std, lat_mu+lat_std,del_s=.01,c='black')
    map_plot.drawgreatcircle(lon_mu+lon_std, lat_mu-lat_std, lon_mu-lon_std, lat_mu+lat_std,del_s=.01,c='black')
    # Plot the GPS data
    x, y = map_plot(lon, lat)
    map_plot.scatter(x, y, marker='o', label=file_name)

plt.xlabel('East-west position (10m)')
plt.ylabel('North-south position (10m)')
plt.title('GPS Data, 3 sensors, 5 minutes')
plt.legend()
plt.savefig('EE 475\GPS_plot.png')
plt.show()
