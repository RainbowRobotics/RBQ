export function decideEntry(i: { hasRobots: boolean; hasAccount: boolean; isDemo: boolean; cloud?: boolean }): '/hub' | '/pin' | '/' {
  if (i.isDemo) return '/';
  if (i.cloud === false) return '/hub';
  return i.hasRobots || i.hasAccount ? '/hub' : '/pin';
}
