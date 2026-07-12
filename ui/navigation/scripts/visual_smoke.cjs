const fs = require('node:fs')
const path = require('node:path')
const { chromium } = require('playwright')

const url = process.env.SLAM_NAV_UI_URL ?? 'http://127.0.0.1:8765/'
const outputDir = process.env.SLAM_NAV_UI_SCREENSHOT_DIR ?? path.resolve('visual-smoke')
const executablePath = process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH
const viewports = [
  { name: 'desktop', width: 1440, height: 900 },
  { name: 'mobile', width: 390, height: 844 },
]

async function main() {
  fs.mkdirSync(outputDir, { recursive: true })
  const browser = await chromium.launch({
    headless: true,
    ...(executablePath ? { executablePath } : {}),
  })

  try {
    for (const viewport of viewports) {
      const page = await browser.newPage({ viewport })
      await page.goto(url, { waitUntil: 'domcontentloaded' })
      await page.waitForTimeout(500)

      const inspectLayout = () => page.evaluate(() => {
        const viewportWidth = window.innerWidth
        const scrollWidth = Math.max(
          document.documentElement.scrollWidth,
          document.body.scrollWidth,
        )
        const overflow = Array.from(document.querySelectorAll('body *'))
          .map((element) => {
            const rect = element.getBoundingClientRect()
            return {
              element: `${element.tagName.toLowerCase()}.${element.className}`,
              left: Math.round(rect.left),
              right: Math.round(rect.right),
              width: Math.round(rect.width),
            }
          })
          .filter((item) => item.right > viewportWidth + 1 || item.left < -1)
          .slice(0, 10)
        return { viewportWidth, scrollWidth, overflow }
      })

      const layout = await inspectLayout()

      await page.screenshot({
        path: path.join(outputDir, `${viewport.name}.png`),
        fullPage: false,
      })

      console.log(`${viewport.name}: viewport=${layout.viewportWidth}, scroll=${layout.scrollWidth}`)
      if (layout.scrollWidth > layout.viewportWidth + 1 || layout.overflow.length) {
        console.error(JSON.stringify(layout.overflow, null, 2))
        throw new Error(`${viewport.name} 页面存在横向溢出`)
      }

      await page.getByRole('button', { name: /任务控制/ }).click()
      await page.getByRole('dialog', { name: '任务控制' }).waitFor()
      const drawerLayout = await inspectLayout()
      await page.screenshot({
        path: path.join(outputDir, `${viewport.name}-tasks.png`),
        fullPage: false,
      })
      await page.close()

      console.log(`${viewport.name} tasks: viewport=${drawerLayout.viewportWidth}, scroll=${drawerLayout.scrollWidth}`)
      if (drawerLayout.scrollWidth > drawerLayout.viewportWidth + 1 || drawerLayout.overflow.length) {
        console.error(JSON.stringify(drawerLayout.overflow, null, 2))
        throw new Error(`${viewport.name} 任务抽屉存在横向溢出`)
      }
    }
  } finally {
    await browser.close()
  }
}

main().catch((error) => {
  console.error(error)
  process.exitCode = 1
})
