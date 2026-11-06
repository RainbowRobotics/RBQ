export type SoundCategory = 'alert' | 'chime' | 'animal' | 'ambient';
export const SOUND_CATEGORIES: { key: SoundCategory; label: string }[] = [
  { key: 'alert', label: '경보·사이렌' },
  { key: 'chime', label: '알림·차임' },
  { key: 'animal', label: '동물' },
  { key: 'ambient', label: '기타' },
];
export type BundledSound = { id: string; label: string; category: SoundCategory; asset: number };

export const SOUNDS: BundledSound[] = [
  { id: 'siren-police', label: '경찰 사이렌', category: 'alert', asset: require('../../assets/sounds/siren-police.mp3') },
  { id: 'siren-civil', label: '민방위 사이렌', category: 'alert', asset: require('../../assets/sounds/siren-civil.mp3') },
  { id: 'siren-emergency', label: '긴급차량 사이렌', category: 'alert', asset: require('../../assets/sounds/siren-emergency.mp3') },
  { id: 'klaxon', label: '클랙슨 경보', category: 'alert', asset: require('../../assets/sounds/klaxon.mp3') },
  { id: 'eas-alert', label: '긴급 경보음', category: 'alert', asset: require('../../assets/sounds/eas-alert.mp3') },
  { id: 'fog-horn', label: '무적', category: 'alert', asset: require('../../assets/sounds/fog-horn.mp3') },
  { id: 'ship-horn', label: '뱃고동', category: 'alert', asset: require('../../assets/sounds/ship-horn.mp3') },
  { id: 'car-horn', label: '자동차 경적', category: 'alert', asset: require('../../assets/sounds/car-horn.mp3') },
  { id: 'whistle-referee', label: '호루라기', category: 'alert', asset: require('../../assets/sounds/whistle-referee.mp3') },
  { id: 'steam-whistle', label: '기적', category: 'alert', asset: require('../../assets/sounds/steam-whistle.mp3') },
  { id: 'doorbell-dingdong', label: '딩동 초인종', category: 'chime', asset: require('../../assets/sounds/doorbell-dingdong.mp3') },
  { id: 'doorbell-electronic', label: '전자 초인종', category: 'chime', asset: require('../../assets/sounds/doorbell-electronic.mp3') },
  { id: 'gong', label: '징', category: 'chime', asset: require('../../assets/sounds/gong.mp3') },
  { id: 'church-bell', label: '교회 종', category: 'chime', asset: require('../../assets/sounds/church-bell.mp3') },
  { id: 'bicycle-bell', label: '자전거 벨', category: 'chime', asset: require('../../assets/sounds/bicycle-bell.mp3') },
  { id: 'beep', label: '삐 소리', category: 'chime', asset: require('../../assets/sounds/beep.mp3') },
  { id: 'dog', label: '개 짖는 소리', category: 'animal', asset: require('../../assets/sounds/dog.mp3') },
  { id: 'wolf', label: '늑대 울음', category: 'animal', asset: require('../../assets/sounds/wolf.mp3') },
  { id: 'rooster', label: '수탉', category: 'animal', asset: require('../../assets/sounds/rooster.mp3') },
  { id: 'eagle', label: '독수리', category: 'animal', asset: require('../../assets/sounds/eagle.mp3') },
  { id: 'hawk', label: '매', category: 'animal', asset: require('../../assets/sounds/hawk.mp3') },
  { id: 'cat', label: '고양이', category: 'animal', asset: require('../../assets/sounds/cat.mp3') },
  { id: 'rain-thunder', label: '비와 천둥', category: 'ambient', asset: require('../../assets/sounds/rain-thunder.mp3') },
  { id: 'applause', label: '박수', category: 'ambient', asset: require('../../assets/sounds/applause.mp3') },
];

