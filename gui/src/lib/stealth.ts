import { rest } from './rest';
import { PROGRAM } from './robotState';

const DAEMON_LAN2CAN_GENERAL_MSG = 600;
const DEV_ID_PDU = 0x510;
const PDU_CMD_DEVICE_SETTING = 0x20;
const PDU_CMD_DEVICE_SETTING_LED = 0x01;
const PDU_CMD_DEVICE_SETTING_BUZZ = 0x02;
const CAN_CH_1 = 1;

function pduDeviceSetting(ip: string, sub: number, enabled: boolean) {
  return rest.commandStruct(ip, PROGRAM.Motion, DAEMON_LAN2CAN_GENERAL_MSG, {
    char: [PDU_CMD_DEVICE_SETTING, sub, enabled ? 1 : 0, 0, 0, 0, 0, 0, 3, CAN_CH_1],
    int: [DEV_ID_PDU],
  });
}

export async function setStealthMode(ip: string, enabled: boolean): Promise<void> {
  await pduDeviceSetting(ip, PDU_CMD_DEVICE_SETTING_LED, enabled);
  await pduDeviceSetting(ip, PDU_CMD_DEVICE_SETTING_BUZZ, enabled);
}
