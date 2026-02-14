import { NextResponse } from 'next/server'
import { redis } from '@/lib/redis'

export async function GET() {
  try {
    // Fetch all data from Redis
    const [songs, images, moods] = await Promise.all([
      redis.get('mood-songs'),
      redis.get('mood-images'),
      redis.get('mood-calendar'),
    ])

    return NextResponse.json({
      songs: songs || [],
      images: images || [],
      moods: moods || {},
    })
  } catch (error) {
    console.error('Error fetching data:', error)
    return NextResponse.json(
      { error: 'Failed to fetch data' },
      { status: 500 }
    )
  }
}

export async function POST(request: Request) {
  try {
    const { type, data } = await request.json()

    // Save to Redis based on type
    await redis.set(`mood-${type}`, data)

    return NextResponse.json({ success: true })
  } catch (error) {
    console.error('Error saving data:', error)
    return NextResponse.json(
      { error: 'Failed to save data' },
      { status: 500 }
    )
  }
}
