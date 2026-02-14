// api/spotify/token.js
// Get Spotify access token using client credentials flow

const CLIENT_ID = process.env.SPOTIFY_CLIENT_ID;
const CLIENT_SECRET = process.env.SPOTIFY_CLIENT_SECRET;

export default async function handler(req, res) {
  // Only allow GET requests
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  // Check if credentials are configured
  if (!CLIENT_ID || !CLIENT_SECRET) {
    return res.status(500).json({ 
      error: 'Spotify credentials not configured',
      message: 'Please set SPOTIFY_CLIENT_ID and SPOTIFY_CLIENT_SECRET in environment variables'
    });
  }

  try {
    // Get access token using client credentials
    const response = await fetch('https://accounts.spotify.com/api/token', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/x-www-form-urlencoded',
        'Authorization': 'Basic ' + Buffer.from(CLIENT_ID + ':' + CLIENT_SECRET).toString('base64'),
      },
      body: 'grant_type=client_credentials'
    });

    if (!response.ok) {
      const error = await response.json();
      console.error('Spotify token error:', error);
      return res.status(response.status).json({ 
        error: 'Failed to get Spotify token',
        details: error
      });
    }

    const data = await response.json();
    
    // Return token with expiry
    res.status(200).json({
      access_token: data.access_token,
      token_type: data.token_type,
      expires_in: data.expires_in
    });
  } catch (error) {
    console.error('Token fetch error:', error);
    res.status(500).json({ 
      error: 'Failed to fetch token',
      message: error.message
    });
  }
}
