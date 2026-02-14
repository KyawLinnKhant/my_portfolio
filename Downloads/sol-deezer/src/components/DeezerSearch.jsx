import React, { useState } from 'react';
import { Search, Music, X, Loader, ExternalLink } from 'lucide-react';

export default function DeezerSearch({ onSelectTrack, onClose }) {
  const [query, setQuery] = useState('');
  const [results, setResults] = useState([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const searchTracks = async () => {
    if (!query.trim()) return;
    
    setLoading(true);
    setError('');
    
    try {
      // Deezer API - NO CREDENTIALS NEEDED!
      const response = await fetch(
        `https://api.deezer.com/search?q=${encodeURIComponent(query)}&limit=20`
      );
      
      if (!response.ok) {
        throw new Error('Search failed');
      }
      
      const data = await response.json();
      setResults(data.data || []);
      
      if (data.data.length === 0) {
        setError('No songs found. Try different keywords!');
      }
    } catch (err) {
      console.error('Deezer search error:', err);
      setError('Failed to search. Please try again!');
    } finally {
      setLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter') {
      searchTracks();
    }
  };

  const handleSelectTrack = (track) => {
    onSelectTrack({
      id: track.id,
      name: track.title,
      artist: track.artist.name,
      album: track.album.title,
      image: track.album.cover_medium,
      previewUrl: track.preview,
      link: track.link
    });
    onClose();
  };

  return (
    <div className="fixed inset-0 bg-black/80 backdrop-blur-sm z-50 flex items-center justify-center p-4">
      <div className="glass rounded-3xl shadow-2xl max-w-2xl w-full max-h-[80vh] flex flex-col border border-purple-500/30">
        {/* Header */}
        <div className="p-6 border-b border-purple-500/30 flex items-center justify-between bg-gradient-to-r from-purple-900/50 to-pink-900/50 rounded-t-3xl">
          <div className="flex items-center gap-3">
            <Music className="w-6 h-6 text-purple-300" />
            <h2 className="text-2xl font-bold">Search Music</h2>
          </div>
          <button
            onClick={onClose}
            className="hover:bg-white/10 p-2 rounded-full transition-all"
          >
            <X className="w-6 h-6" />
          </button>
        </div>

        {/* Search Bar */}
        <div className="p-6 border-b border-purple-500/30">
          <div className="flex gap-2">
            <div className="flex-1 relative">
              <Search className="absolute left-3 top-1/2 -translate-y-1/2 w-5 h-5 text-purple-400" />
              <input
                type="text"
                value={query}
                onChange={(e) => setQuery(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Search for songs, artists..."
                className="w-full pl-10 pr-4 py-3 bg-purple-950/50 border border-purple-500/30 rounded-xl focus:border-purple-500 focus:outline-none text-white placeholder-purple-400"
                autoFocus
              />
            </div>
            <button
              onClick={searchTracks}
              disabled={loading || !query.trim()}
              className="bg-gradient-to-r from-purple-600 to-pink-600 text-white px-6 py-3 rounded-xl font-semibold hover:from-purple-700 hover:to-pink-700 transition-all disabled:opacity-50 disabled:cursor-not-allowed flex items-center gap-2"
            >
              {loading ? (
                <>
                  <Loader className="w-5 h-5 animate-spin" />
                  Searching...
                </>
              ) : (
                <>
                  <Search className="w-5 h-5" />
                  Search
                </>
              )}
            </button>
          </div>
          {error && (
            <p className="text-red-400 text-sm mt-2">{error}</p>
          )}
        </div>

        {/* Results */}
        <div className="flex-1 overflow-y-auto p-6">
          {results.length === 0 && !loading && (
            <div className="text-center py-12 text-purple-300">
              <Music className="w-16 h-16 mx-auto mb-4 opacity-50" />
              <p className="text-lg">Search for your favorite songs!</p>
              <p className="text-sm mt-2 opacity-70">Try: "Lover Taylor Swift" or "Blinding Lights"</p>
              <p className="text-xs mt-4 opacity-50">✨ Powered by Deezer - No setup needed!</p>
            </div>
          )}

          <div className="space-y-3">
            {results.map((track) => (
              <div
                key={track.id}
                onClick={() => handleSelectTrack(track)}
                className="flex items-center gap-4 p-4 bg-purple-950/30 hover:bg-purple-900/50 rounded-xl cursor-pointer transition-all border border-purple-500/0 hover:border-purple-500/50 group"
              >
                {track.album?.cover_medium && (
                  <img
                    src={track.album.cover_medium}
                    alt={track.title}
                    className="w-16 h-16 rounded-lg object-cover ring-2 ring-purple-500/0 group-hover:ring-purple-500/50 transition-all"
                  />
                )}
                <div className="flex-1 min-w-0">
                  <h3 className="font-semibold text-white truncate">
                    {track.title}
                  </h3>
                  <p className="text-sm text-purple-300 truncate">
                    {track.artist.name}
                  </p>
                  {track.album?.title && (
                    <p className="text-xs text-purple-400 truncate mt-1">
                      {track.album.title}
                    </p>
                  )}
                </div>
                <Music className="w-5 h-5 text-purple-400 group-hover:text-purple-300 transition-colors" />
              </div>
            ))}
          </div>
        </div>

        {/* Footer */}
        <div className="p-4 border-t border-purple-500/30 bg-purple-950/30 rounded-b-3xl">
          <p className="text-xs text-purple-300 text-center flex items-center justify-center gap-2">
            <Music className="w-3 h-3" />
            Preview & play songs with 30-second clips
          </p>
        </div>
      </div>
    </div>
  );
}
