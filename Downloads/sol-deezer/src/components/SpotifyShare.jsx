import React, { useState } from 'react';
import { Music, X, ExternalLink, Link as LinkIcon } from 'lucide-react';

export default function SpotifyShare({ onSelectTrack, onClose }) {
  const [trackUrl, setTrackUrl] = useState('');
  const [error, setError] = useState('');

  const handleOpenSpotify = () => {
    // Open Spotify app to get shareable link
    window.open('spotify://', '_blank');
    
    // Also open web version as fallback
    setTimeout(() => {
      window.open('https://open.spotify.com/', '_blank');
    }, 500);
  };

  const handlePasteLink = async () => {
    try {
      const text = await navigator.clipboard.readText();
      setTrackUrl(text);
      processSpotifyLink(text);
    } catch (err) {
      setError('Could not read clipboard. Please paste manually.');
    }
  };

  const processSpotifyLink = (url) => {
    setError('');
    
    // Extract track ID from Spotify URL
    // Formats: 
    // https://open.spotify.com/track/TRACK_ID
    // spotify:track:TRACK_ID
    
    let trackId = null;
    
    if (url.includes('open.spotify.com/track/')) {
      trackId = url.split('open.spotify.com/track/')[1]?.split('?')[0];
    } else if (url.includes('spotify:track:')) {
      trackId = url.split('spotify:track:')[1];
    }
    
    if (trackId) {
      // For now, use a simple track object
      // In production, you'd fetch track details from Spotify API
      onSelectTrack({
        id: trackId,
        name: 'Song from Spotify',
        artist: 'Artist',
        album: '',
        image: '',
        uri: `spotify:track:${trackId}`,
        previewUrl: null
      });
      onClose();
    } else {
      setError('Invalid Spotify link. Please copy a song link from Spotify.');
    }
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    if (trackUrl.trim()) {
      processSpotifyLink(trackUrl);
    }
  };

  return (
    <div className="fixed inset-0 bg-black/80 backdrop-blur-sm z-50 flex items-center justify-center p-4">
      <div className="glass rounded-3xl shadow-2xl max-w-md w-full border border-purple-500/30">
        {/* Header */}
        <div className="p-6 border-b border-purple-500/30 flex items-center justify-between bg-gradient-to-r from-purple-900/50 to-pink-900/50 rounded-t-3xl">
          <div className="flex items-center gap-3">
            <Music className="w-6 h-6 text-green-400" />
            <h2 className="text-2xl font-bold">Add Spotify Song</h2>
          </div>
          <button
            onClick={onClose}
            className="hover:bg-white/10 p-2 rounded-full transition-all"
          >
            <X className="w-6 h-6" />
          </button>
        </div>

        {/* Content */}
        <div className="p-6">
          <div className="mb-6">
            <h3 className="text-lg font-semibold mb-3 text-purple-200">How to add a song:</h3>
            <ol className="space-y-3 text-sm text-purple-300">
              <li className="flex items-start gap-2">
                <span className="bg-purple-600 text-white rounded-full w-6 h-6 flex items-center justify-center flex-shrink-0 text-xs font-bold">1</span>
                <span>Open Spotify and find the song you want</span>
              </li>
              <li className="flex items-start gap-2">
                <span className="bg-purple-600 text-white rounded-full w-6 h-6 flex items-center justify-center flex-shrink-0 text-xs font-bold">2</span>
                <span>Tap the <strong>Share</strong> button → <strong>Copy Song Link</strong></span>
              </li>
              <li className="flex items-start gap-2">
                <span className="bg-purple-600 text-white rounded-full w-6 h-6 flex items-center justify-center flex-shrink-0 text-xs font-bold">3</span>
                <span>Come back here and paste the link below</span>
              </li>
            </ol>
          </div>

          {/* Open Spotify Button */}
          <button
            onClick={handleOpenSpotify}
            className="w-full bg-green-600 hover:bg-green-700 text-white py-3 rounded-xl font-semibold transition-all flex items-center justify-center gap-2 mb-4"
          >
            <ExternalLink className="w-5 h-5" />
            Open Spotify App
          </button>

          {/* Paste Link Form */}
          <form onSubmit={handleSubmit} className="space-y-3">
            <div className="relative">
              <input
                type="text"
                value={trackUrl}
                onChange={(e) => setTrackUrl(e.target.value)}
                placeholder="Paste Spotify link here..."
                className="w-full px-4 py-3 bg-purple-950/50 border border-purple-500/30 rounded-xl focus:border-purple-500 focus:outline-none text-white placeholder-purple-400 pr-24"
              />
              <button
                type="button"
                onClick={handlePasteLink}
                className="absolute right-2 top-1/2 -translate-y-1/2 bg-purple-600 hover:bg-purple-700 px-3 py-1.5 rounded-lg text-sm transition-all"
              >
                Paste
              </button>
            </div>
            
            {error && (
              <p className="text-red-400 text-sm">{error}</p>
            )}

            <button
              type="submit"
              disabled={!trackUrl.trim()}
              className="w-full bg-gradient-to-r from-purple-600 to-pink-600 text-white py-3 rounded-xl font-semibold hover:from-purple-700 hover:to-pink-700 transition-all disabled:opacity-50 disabled:cursor-not-allowed flex items-center justify-center gap-2"
            >
              <LinkIcon className="w-5 h-5" />
              Add Song to Memory
            </button>
          </form>

          {/* Example */}
          <div className="mt-4 p-3 bg-purple-950/30 rounded-lg border border-purple-500/20">
            <p className="text-xs text-purple-300 mb-1">Example Spotify link:</p>
            <code className="text-xs text-purple-400 break-all">
              https://open.spotify.com/track/abc123...
            </code>
          </div>
        </div>
      </div>
    </div>
  );
}
