// api/posts/get.js
// Get all posts (shared between users)

export default async function handler(req, res) {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    // Check for Upstash Redis environment variables (primary)
    if (process.env.UPSTASH_REDIS_REST_URL && process.env.UPSTASH_REDIS_REST_TOKEN) {
      const { Redis } = await import('@upstash/redis');
      const redis = new Redis({
        url: process.env.UPSTASH_REDIS_REST_URL,
        token: process.env.UPSTASH_REDIS_REST_TOKEN,
      });
      
      const hisPostsRaw = await redis.get('sol_posts_his');
      const hersPostsRaw = await redis.get('sol_posts_hers');
      
      const hisPosts = hisPostsRaw 
        ? (typeof hisPostsRaw === 'string' ? JSON.parse(hisPostsRaw) : hisPostsRaw)
        : [];
      const hersPosts = hersPostsRaw 
        ? (typeof hersPostsRaw === 'string' ? JSON.parse(hersPostsRaw) : hersPostsRaw)
        : [];
      
      console.log(`[Upstash] Retrieved ${hisPosts.length} posts for his, ${hersPosts.length} posts for hers`);
      
      return res.status(200).json({
        his: hisPosts,
        hers: hersPosts,
        provider: 'upstash'
      });
    }
    
    // Fallback to Vercel KV if configured
    if (process.env.KV_REST_API_URL && process.env.KV_REST_API_TOKEN) {
      const { kv } = await import('@vercel/kv');
      
      const hisPostsRaw = await kv.get('sol_posts_his');
      const hersPostsRaw = await kv.get('sol_posts_hers');
      
      const hisPosts = hisPostsRaw 
        ? (typeof hisPostsRaw === 'string' ? JSON.parse(hisPostsRaw) : hisPostsRaw)
        : [];
      const hersPosts = hersPostsRaw 
        ? (typeof hersPostsRaw === 'string' ? JSON.parse(hersPostsRaw) : hersPostsRaw)
        : [];
      
      console.log(`[Vercel KV] Retrieved ${hisPosts.length} posts for his, ${hersPosts.length} posts for hers`);
      
      return res.status(200).json({
        his: hisPosts,
        hers: hersPosts,
        provider: 'vercel-kv'
      });
    }
    
    // No database configured
    return res.status(503).json({ 
      error: 'Database not configured',
      message: 'Please set up Upstash Redis integration in Vercel dashboard'
    });
  } catch (error) {
    console.error('Get posts error:', error);
    res.status(500).json({ 
      error: 'Failed to get posts',
      message: error.message 
    });
  }
}
