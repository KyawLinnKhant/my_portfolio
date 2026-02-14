import type { Metadata } from 'next'
import './globals.css'

export const metadata: Metadata = {
  title: 'Mood Board - Track Your Moments Together',
  description: 'A shared space for couples to track moods, music, and memories',
}

export default function RootLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  )
}
