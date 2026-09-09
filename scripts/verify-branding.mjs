import fs from 'node:fs'
import path from 'node:path'

const root = process.cwd()
const ignored = new Set(['.git', 'node_modules', 'dist'])
const forbidden = ('wi' + 'sp').toLowerCase()
const textExtensions = new Set(['.ts', '.tsx', '.js', '.mjs', '.json', '.md', '.html', '.css', '.yml', '.yaml', '.svg', '.rs', '.toml'])
const violations = []

function ignoreDirectory(directory, name) {
  if (ignored.has(name) || name.startsWith('.edge-')) return true
  return name === 'target' && path.basename(directory) === 'core'
}

function walk(directory) {
  for (const entry of fs.readdirSync(directory, { withFileTypes: true })) {
    if (entry.isDirectory() && ignoreDirectory(directory, entry.name)) continue
    const full = path.join(directory, entry.name)
    if (entry.isDirectory()) { walk(full); continue }
    if (!textExtensions.has(path.extname(entry.name).toLowerCase())) continue
    const content = fs.readFileSync(full, 'utf8').toLowerCase()
    if (content.includes(forbidden)) violations.push(path.relative(root, full))
  }
}

walk(root)
if (violations.length) {
    console.error(`Forbidden legacy-brand reference found in: ${violations.join(', ')}`)
    process.exit(1)
}
console.log('Brand isolation verified.')
