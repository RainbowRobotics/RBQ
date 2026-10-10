import type { WidgetCase } from '../types';

export const live: WidgetCase[] = [
  { name: 'CameraView',
    code: `import { CameraView } from '@/components/CameraView';

<View style={{ flex: 1 }}>
  <CameraView streamId={0} />   // 0=전방, 소스 전환은 streamId 로만
</View>
// 마운트되면 스스로 webrtcClient.connect(ip) 를 부른다 — 영상이 필요 없는 화면에 두지 말 것.`, from: '@/components/CameraView', unavailable: '전역 스토어를 가짜로 덮어야만 뜨는 위젯이라 **갤러리에서는 전시하지 않는다.** 덮어 두면 그 값이 앱 전역에 남아 실제 세션을 오염시키고(접근 레벨 하락·연결 끊김·가짜 Wi-Fi 목록), 되돌리기를 붙여도 경로 하나씩 계속 샜다. 실물은 실제 화면에서 보고, 여기서는 아래 코드와 규약만 가져간다.',
    when: '로봇 카메라 영상. 마운트되면 스스로 webrtcClient.connect(ip) 를 불러 실제 로봇과 협상하므로, 영상이 필요 없는 화면에 두지 않는다.' },
  { name: 'WifiPicker',
    code: `import { WifiPicker } from '@/components/WifiPicker';

{open && <WifiPicker onClose={() => setOpen(false)} />}
// 스캔·연결은 useWifi 스토어가 데스크탑(Tauri) 브리지를 통해 한다. 웹에선 목록이 빈다.`, from: '@/components/WifiPicker', unavailable: '전역 스토어를 가짜로 덮어야만 뜨는 위젯이라 **갤러리에서는 전시하지 않는다.** 덮어 두면 그 값이 앱 전역에 남아 실제 세션을 오염시키고(접근 레벨 하락·연결 끊김·가짜 Wi-Fi 목록), 되돌리기를 붙여도 경로 하나씩 계속 샜다. 실물은 실제 화면에서 보고, 여기서는 아래 코드와 규약만 가져간다.',
    when: 'AP 목록·비밀번호 입력. 스캔·연결을 useWifi 스토어가 데스크탑(Tauri) 브리지로 수행하므로 웹에서는 목록이 비고, 실물을 띄우려면 그 스토어를 덮어야 한다.' },
];
