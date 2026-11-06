import { PermissionsAndroid, Platform } from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import NetInfo from '@react-native-community/netinfo';

const KEY = 'rbq-location-asked';

export async function askLocationOnce(): Promise<void> {
  if (Platform.OS !== 'android') return;
  try {
    const perm = PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION;
    if (await PermissionsAndroid.check(perm)) return;
    if (await AsyncStorage.getItem(KEY)) return;
    await AsyncStorage.setItem(KEY, '1');
    const r = await PermissionsAndroid.request(perm);
    if (r === 'granted') NetInfo.refresh().catch(() => {});
  } catch { }
}
