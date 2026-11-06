import { router, type Href } from 'expo-router';

export function goTop(path: Href) {
  if (router.canDismiss()) router.dismissAll();
  router.replace(path);
}
