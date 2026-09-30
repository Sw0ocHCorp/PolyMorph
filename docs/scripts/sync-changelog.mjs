// Copies the repository's CHANGELOG.md into the site as the /changelog page.
//
// The root CHANGELOG.md stays the single source of truth (readable on GitHub, written in
// every commit). The copy under src/pages/ is generated before `start` and `build`, and
// git-ignored. It is a `.md` file, which `markdown.format: 'detect'` parses as plain
// CommonMark: a `<` or `{` in a changelog entry cannot break the build.
//
// While `npm run start` is running, edits to CHANGELOG.md are not picked up: run
// `npm run sync-changelog` (or restart) to refresh the page.

import {readFileSync, writeFileSync} from 'node:fs';
import {dirname, resolve} from 'node:path';
import {fileURLToPath} from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const source = resolve(here, '../../CHANGELOG.md');
const target = resolve(here, '../src/pages/changelog.md');

const frontMatter = [
  '---',
  'title: Changelog',
  'description: Every commit of PolyMorph, newest first, grouped by phase.',
  // Phases only in the table of contents: one line per commit would be unreadable.
  'toc_max_heading_level: 2',
  '---',
  '',
  '',
].join('\n');

writeFileSync(target, frontMatter + readFileSync(source, 'utf8'));
console.log(`[sync-changelog] ${source} -> ${target}`);
