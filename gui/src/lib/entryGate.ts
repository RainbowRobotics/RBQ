export function decideEntry(i: { isDemo: boolean; route?: string }): string {
  if (i.isDemo) return '/';
  return i.route ?? '/hub';
}
