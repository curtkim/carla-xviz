import glob
import os
import sys
import shutil

import numpy as np
import carla

import xviz_avs as xviz
import xviz_avs.io as xi
import xviz_avs.builder as xbuilder

from carla_sync_mode import CarlaSyncMode


def get_metadata():
    builder = xviz.XVIZMetadataBuilder()
    builder.stream("/vehicle_pose").category(xviz.CATEGORY.POSE)
    builder.stream("/circle") \
        .coordinate(xviz.COORDINATE_TYPES.IDENTITY) \
        .stream_style({'fill_color': [200, 0, 70, 128]}) \
        .category(xviz.CATEGORY.PRIMITIVE) \
        .type(xviz.PRIMITIVE_TYPES.CIRCLE)
    builder.stream("/points") \
        .coordinate(xviz.COORDINATE_TYPES.VEHICLE_RELATIVE) \
        .category(xviz.CATEGORY.PRIMITIVE) \
        .type(xviz.PRIMITIVE_TYPES.POINT) \
        .stream_style({
            'radius_pixels': 6
        })
    return builder.get_message()

def get_message(metadata, frame, vehicle):
    timestamp = frame / 30.0

    location = vehicle.get_transform().location
    rotation = vehicle.get_transform().rotation

    builder = xviz.XVIZBuilder(metadata=metadata)
    builder.pose() \
        .timestamp(timestamp) \
        .orientation(rotation.roll, rotation.pitch, rotation.yaw) \
        .position(location.x, location.y, location.z)

    builder.primitive('/points') \
        .points([3, 0, 0, 0, 3, 0, 0, 0, 3]) \
        .colors([200, 40, 80, 80, 40, 200, 80, 200, 40]) \
        .id("indicator")
    return builder.get_message()


def load_world_if_needed(client, map_name):
    if not map_name.endswith(client.get_world().get_map().name):
        client.load_world(map_name)
    else:
        print('current map is already ', map_name)


OUTPUT = "output"

def main():
    if os.path.exists(OUTPUT):
        shutil.rmtree(OUTPUT)
    os.mkdir(OUTPUT)

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        load_world_if_needed(client, "/Game/Carla/Maps/Town01")

        world = client.get_world()
        map = world.get_map()

        # 시작위치와 방향
        start_tf = carla.Transform(carla.Location(x=230, y=55.4, z=0.1), carla.Rotation(0, 180, 0))
        start_wp = map.get_waypoint(start_tf.location)

        blueprint_library = world.get_blueprint_library()
        bp = blueprint_library.find('vehicle.jeep.wrangler_rubicon')

        vehicle = world.spawn_actor(bp, start_tf)
        vehicle.set_autopilot(True)
        print('created %s' % vehicle.type_id)
        # vehicle.set_velocity(carla.Vector3D(-10, 0, 0)) # 초기 속도를 지정할 수 있다.

        camera_rgb = world.spawn_actor(
            blueprint_library.find('sensor.camera.rgb'),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            attach_to=vehicle)

        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', str(200))
        lidar_bp.set_attribute('channels', str(32))
        lidar_bp.set_attribute('rotation_frequency', str(30))
        lidar_sensor = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(x=1.5, y=0, z=2.4)),
                                         attach_to=vehicle)

        writer = xi.XVIZJsonWriter(xi.DirectorySource("output"))
        metadata = get_metadata()
        writer.write_message(metadata)

        with CarlaSyncMode(world, camera_rgb, lidar_sensor, fps=30) as sync_mode:
            while True:
                curr_tf = vehicle.get_transform()
                curr_location = vehicle.get_location()

                snapshot, image_rgb, pointcloud = sync_mode.tick(timeout=2.0)
                print(snapshot.frame)

                spectator_loc = carla.Location(curr_location.x, curr_location.y, curr_location.z + 2.5);
                world.get_spectator().set_transform(carla.Transform(spectator_loc,curr_tf.rotation))
                writer.write_message(get_message(metadata, snapshot.frame, vehicle))

        print("success")

    finally:
        writer.close()
        print('writer closed')

        lidar_sensor.destroy()
        camera_rgb.destroy()

        print('destroying vehicle')
        vehicle.destroy()

        print('done.')


if __name__ == '__main__':
    main()