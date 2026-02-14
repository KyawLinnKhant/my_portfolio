import { Redis } from '@upstash/redis'

// Initialize Upstash Redis client
// These environment variables will be set in Vercel
export const redis = new Redis({
  url: process.env.UPSTASH_REDIS_REST_URL!,
  token: process.env.UPSTASH_REDIS_REST_TOKEN!,
})
