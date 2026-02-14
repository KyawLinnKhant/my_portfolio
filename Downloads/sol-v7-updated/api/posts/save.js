// api/posts/save.js
// Save posts for a user

export default async function handler(req, res) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { userSide, posts } = req.body;
    
    if (!userSide || !posts) {
      return res.status(400).json({ error: 'Missing userSide or posts' });
    }

    // Check for Upstash Redis environment variables (primary)
    if (process.env.UPSTASH_REDIS_REST_URL && process.env.UPSTASH_REDIS_REST_TOKEN) {
      const { Redis } = await import('@upstash/redis');
      const redis = new Redis({
        url: process.env.UPSTASH_REDIS_REST_URL,
        token: process.env.UPSTASH_REDIS_REST_TOKEN,
      });
      
      await redis.set(`sol_posts_${userSide}`, JSON.stringify(posts));
      console.log(`[Upstash] Saved ${posts.length} posts for ${userSide}`);
      
      return res.status(200).json({ 
        success: true,
        count: posts.length,
        provider: 'upstash'
      });
    }
    
    // Fallback to Vercel KV if configured
    if (process.env.KV_REST_API_URL && process.env.KV_REST_API_TOKEN) {
      const { kv } = await import('@vercel/kv');
      await kv.set(`sol_posts_${userSide}`, JSON.stringify(posts));
      console.log(`[Vercel KV] Saved ${posts.length} posts for ${userSide}`);
      
      return res.status(200).json({ 
        success: true,
        count: posts.length,
        provider: 'vercel-kv'
      });
    }
    
    // No database configured
    return res.status(503).json({ 
      error: 'Database not configured',
      message: 'Please set up Upstash Redis integration in Vercel dashboard'
    });
  } catch (error) {
    console.error('Save posts error:', error);
    res.status(500).json({ 
      error: 'Failed to save posts',
      message: error.message 
    });
  }
}
