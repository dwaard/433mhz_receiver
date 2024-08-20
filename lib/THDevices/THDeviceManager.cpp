#include "THDeviceManager.h"

/**
 * Creates a new instance of this class. The max allowed amount must
 * be specified in order to limit the amount of memeory that is
 * allocated.
 * 
 * @param maxDevices the maximum llowed devices for this manager. 
 */
THDeviceManager::THDeviceManager(uint8_t maxDevices) {
  maxSize = maxDevices;
  THDevice* newArr = (THDevice*) std::malloc(sizeof(THDevice*) * maxDevices);
  delete devices;
  devices = newArr;
  size=0;
}

/**
 * Adds the specified device to this manager. If the device already 
 * exists, or the size of the device list is already at its maximum,
 * nothing will happen.
 * 
 * @param device The device to add
 */
void THDeviceManager::add(THDevice device) {
  if (hasDevice(device) || size >= maxSize)
    return;
  devices[size++] = device;
}

/**
 * Checks if a device with the given deviceID is present in this manager.
 * If so, `true` is returned, otherwise `false`.
 * 
 * @param deviceID the ID of the device.
 */
bool THDeviceManager::hasDevice(uint8_t deviceID) {
  for (int n=0; n<size; n++) {
    if (devices[n].hasID(deviceID))
      return true;
  }
  return false;
}

/**
 * Checks if a device is present in this manager. If so, `true` is returned, 
 * otherwise `false`.
 * 
 * @param device the device.
 */
bool THDeviceManager::hasDevice(THDevice device) {
  for (int n=0; n<size; n++) {
    if (devices[n] == device)
      return true;
  }
  return false;
}

/**
 * Returns a pointer to the device with the given `deviceID`. If this
 * manager does not have a matching device, some undeined state will 
 * return.
 * 
 * @param deviceID the ID of the device.
 */
THDevice *THDeviceManager::getDevice(uint8_t deviceID) {
  for (int n=0; n<size; n++) {
    if (devices[n].hasID(deviceID))
      return &devices[n];
  }
  return nullptr;
}

/**
 * Process a new measurement for one of its devices. Returns `true` if 
 * and only if this method results into a state change of one of the
 * attached devices. Otherwise `false`.
 * 
 * @param measurement the measurement to process
 */
bool THDeviceManager::process(Measurement m) {
  for (int n=0; n<size; n++) {
    if (devices[n].hasID(m.deviceID)) {
      return devices[n].process(m);
    }
  }
  return false;
}