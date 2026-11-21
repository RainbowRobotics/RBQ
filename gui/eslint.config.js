const tseslint = require('typescript-eslint');
const reactHooks = require('eslint-plugin-react-hooks');

const FACTORY_HOOK = {
  selector: [
    ':function VariableDeclarator[id.name=/^use[A-Z]/] > :function :function',
    ':function Property[key.name=/^use[A-Z]/] > :function :function',
    ':function FunctionDeclaration[id.name=/^use[A-Z]/] :function',
  ].join(', '),
  message: '함수 안에서 만든 훅에 콜백을 두지 마세요 — 훅 본체는 모듈 맨 위 함수로, 값은 인자로(gui/CLAUDE.md React Compiler).',
};
const BACK_HANDLER = {
  selector: "MemberExpression[object.name='BackHandler'][property.name='addEventListener']",
  message: '뒤로 입력은 components/BackNav 한 곳에서만 — 갈 곳은 lib/backNav.ts, 패드 B 는 usePadBackOff().',
};

const rnModalOrientations = {
  meta: { type: 'problem', messages: { missing: 'react-native Modal 에 supportedOrientations={MODAL_ORIENTATIONS} 를 넘기거나 ui/overlays 의 Modal 을 쓰세요(없으면 iOS SIGABRT).' } },
  create(context) {
    const local = new Set();
    return {
      ImportDeclaration(node) {
        if (node.source.value !== 'react-native') return;
        for (const s of node.specifiers) if (s.type === 'ImportSpecifier' && s.imported.name === 'Modal') local.add(s.local.name);
      },
      JSXOpeningElement(node) {
        if (node.name.type !== 'JSXIdentifier' || !local.has(node.name.name)) return;
        if (!node.attributes.some((a) => a.type === 'JSXAttribute' && a.name.name === 'supportedOrientations')) context.report({ node, messageId: 'missing' });
      },
    };
  },
};

module.exports = [
  {
    files: ['src/**/*.{ts,tsx}'],
    languageOptions: { parser: tseslint.parser },
    linterOptions: { reportUnusedDisableDirectives: 'off' },
    plugins: { 'react-hooks': reactHooks, rbq: { rules: { 'rn-modal-orientations': rnModalOrientations } } },
    rules: {
      'react-hooks/void-use-memo': 'error',
      'react-hooks/globals': 'error',
      'react-hooks/error-boundaries': 'error',
      'react-hooks/set-state-in-render': 'error',
      'react-hooks/incompatible-library': 'error',
      'react-hooks/unsupported-syntax': 'error',
      'react-hooks/config': 'error',
      'react-hooks/gating': 'error',
      'rbq/rn-modal-orientations': 'error',
      'no-restricted-syntax': ['error', FACTORY_HOOK, BACK_HANDLER],
    },
  },
  { files: ['src/components/BackNav.tsx'], rules: { 'no-restricted-syntax': ['error', FACTORY_HOOK] } },
];
