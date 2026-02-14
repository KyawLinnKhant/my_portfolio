import React, { useState, useEffect, useRef } from 'react';
import { Heart, Camera, MessageSquare, Image as ImageIcon, X, LogOut, Trash2, Music, ExternalLink, Calendar as CalendarIcon, Send } from 'lucide-react';
import DeezerSearch from './components/DeezerSearch';
import Calendar from './components/Calendar';

export default function SOL() {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [userSide, setUserSide] = useState(null);
  const [password, setPassword] = useState('');
  const [posts, setPosts] = useState([]);
  const [showUpload, setShowUpload] = useState(false);
  const [newPost, setNewPost] = useState({ type: 'photo', content: '', caption: '', musicTrack: null });
  const [loading, setLoading] = useState(true);
  const [showMusicSearch, setShowMusicSearch] = useState(false);
  const [showCalendar, setShowCalendar] = useState(false);
  const [selectedDate, setSelectedDate] = useState(null);
  const messagesEndRef = useRef(null);

  const isArtifact = typeof window !== 'undefined' && window.storage;

  useEffect(() => {
    if (isAuthenticated) {
      loadPosts();
    }
  }, [isAuthenticated]);

  useEffect(() => {
    scrollToBottom();
  }, [posts]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const loadPosts = async () => {
    try {
      setLoading(true);
      let allPosts = [];
      
      if (isArtifact) {
        // Load both his and hers posts
        const hisResult = await window.storage.get(`sol_posts_his`, true);
        const hersResult = await window.storage.get(`sol_posts_hers`, true);
        
        const hisPosts = hisResult && hisResult.value ? JSON.parse(hisResult.value) : [];
        const hersPosts = hersResult && hersResult.value ? JSON.parse(hersResult.value) : [];
        
        allPosts = [...hisPosts, ...hersPosts];
      } else {
        const hisStored = localStorage.getItem(`sol_posts_his`);
        const hersStored = localStorage.getItem(`sol_posts_hers`);
        
        const hisPosts = hisStored ? JSON.parse(hisStored) : [];
        const hersPosts = hersStored ? JSON.parse(hersStored) : [];
        
        allPosts = [...hisPosts, ...hersPosts];
      }
      
      // Sort by timestamp
      allPosts.sort((a, b) => new Date(a.timestamp) - new Date(b.timestamp));
      
      setPosts(allPosts);
    } catch (error) {
      console.log('No posts yet or error loading:', error);
      setPosts([]);
    } finally {
      setLoading(false);
    }
  };

  const handleLogin = (e) => {
    e.preventDefault();
    if (password === '26kylikh') {
      setUserSide('his');
      setIsAuthenticated(true);
    } else if (password === '3o3Pan') {
      setUserSide('hers');
      setIsAuthenticated(true);
    } else {
      alert('Incorrect password 💔');
    }
  };

  const handleLogout = () => {
    setIsAuthenticated(false);
    setUserSide(null);
    setPassword('');
    setPosts([]);
  };

  const handleImageUpload = (e) => {
    const file = e.target.files[0];
    if (file) {
      if (file.size > 5 * 1024 * 1024) {
        alert('Image too large! Please choose a smaller image (under 5MB)');
        return;
      }
      
      const reader = new FileReader();
      reader.onloadend = () => {
        setNewPost({ ...newPost, content: reader.result });
      };
      reader.readAsDataURL(file);
    }
  };

  const handleSelectMusicTrack = (track) => {
    setNewPost({ ...newPost, musicTrack: track });
  };

  const handleSubmitPost = async () => {
    if (!newPost.content && newPost.type === 'photo') {
      alert('Please add a photo first! 📸');
      return;
    }
    if (!newPost.content && newPost.type === 'message') {
      alert('Please write a message first! 💌');
      return;
    }

    const post = {
      id: Date.now(),
      type: newPost.type,
      content: newPost.content,
      caption: newPost.caption,
      musicTrack: newPost.musicTrack,
      timestamp: new Date().toISOString(),
      from: userSide
    };

    try {
      let existingPosts = [];
      
      if (isArtifact) {
        const result = await window.storage.get(`sol_posts_${userSide}`, true);
        if (result && result.value) {
          existingPosts = JSON.parse(result.value);
        }
        
        const updatedPosts = [post, ...existingPosts];
        await window.storage.set(`sol_posts_${userSide}`, JSON.stringify(updatedPosts), true);
      } else {
        const stored = localStorage.getItem(`sol_posts_${userSide}`);
        if (stored) {
          existingPosts = JSON.parse(stored);
        }
        
        const updatedPosts = [post, ...existingPosts];
        localStorage.setItem(`sol_posts_${userSide}`, JSON.stringify(updatedPosts));
      }
      
      setNewPost({ type: 'photo', content: '', caption: '', musicTrack: null });
      setShowUpload(false);
      
      loadPosts();
    } catch (error) {
      console.error('Error saving post:', error);
      alert('Failed to post. Try again! 💔');
    }
  };

  const handleDeletePost = async (postId) => {
    if (!confirm('Delete this memory? 🥺')) return;

    try {
      // Find which side the post belongs to
      const post = posts.find(p => p.id === postId);
      if (!post) return;

      const targetSide = post.from;
      let existingPosts = [];

      if (isArtifact) {
        const result = await window.storage.get(`sol_posts_${targetSide}`, true);
        if (result && result.value) {
          existingPosts = JSON.parse(result.value);
        }
        const updatedPosts = existingPosts.filter(p => p.id !== postId);
        await window.storage.set(`sol_posts_${targetSide}`, JSON.stringify(updatedPosts), true);
      } else {
        const stored = localStorage.getItem(`sol_posts_${targetSide}`);
        if (stored) {
          existingPosts = JSON.parse(stored);
        }
        const updatedPosts = existingPosts.filter(p => p.id !== postId);
        localStorage.setItem(`sol_posts_${targetSide}`, JSON.stringify(updatedPosts));
      }

      // Reload all posts
      loadPosts();
    } catch (error) {
      console.error('Error deleting post:', error);
      alert('Failed to delete');
    }
  };

  const handleDateSelect = (date) => {
    setSelectedDate(date);
  };

  const handleClearFilter = () => {
    setSelectedDate(null);
  };

  // Filter posts by selected date
  const filteredPosts = selectedDate
    ? posts.filter(post => {
        const postDate = new Date(post.timestamp);
        return (
          postDate.getDate() === selectedDate.getDate() &&
          postDate.getMonth() === selectedDate.getMonth() &&
          postDate.getFullYear() === selectedDate.getFullYear()
        );
      })
    : posts;

  // Login Screen
  if (!isAuthenticated) {
    return (
      <div className="min-h-screen pansy-bg flex items-center justify-center p-4">
        <div className="absolute top-10 left-10 text-6xl opacity-20">🌸</div>
        <div className="absolute top-20 right-20 text-5xl opacity-15">💜</div>
        <div className="absolute bottom-20 left-20 text-5xl opacity-15">🌺</div>
        <div className="absolute bottom-10 right-10 text-6xl opacity-10">✨</div>
        
        <div className="glass rounded-3xl shadow-2xl p-8 max-w-md w-full border border-purple-500/30">
          <div className="text-center mb-8">
            <div className="text-7xl mb-4">💜</div>
            <h1 className="text-5xl font-bold bg-gradient-to-r from-purple-400 to-pink-400 bg-clip-text text-transparent mb-2">
              SOL
            </h1>
            <p className="text-purple-300 italic text-lg">Our Little Garden</p>
            <p className="text-purple-400 text-sm mt-2">Where memories bloom with music 🌸🎵</p>
          </div>
          
          <form onSubmit={handleLogin} className="space-y-4">
            <input
              type="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="Enter your secret code..."
              className="w-full px-4 py-3 rounded-2xl bg-purple-950/50 border border-purple-500/30 focus:border-purple-500 focus:outline-none text-center text-lg text-white placeholder-purple-400"
            />
            <button
              type="submit"
              className="w-full bg-gradient-to-r from-purple-600 to-pink-600 text-white py-3 rounded-2xl font-semibold hover:from-purple-700 hover:to-pink-700 transition-all shadow-lg flex items-center justify-center gap-2"
            >
              <Heart className="w-5 h-5" />
              Enter Our Garden
            </button>
          </form>
        </div>
      </div>
    );
  }

  const sideColors = userSide === 'his' 
    ? { primary: 'from-blue-600 to-purple-600', bubble: 'bg-gradient-to-br from-blue-600 to-purple-600' }
    : { primary: 'from-pink-600 to-purple-600', bubble: 'bg-gradient-to-br from-pink-600 to-purple-600' };

  return (
    <div className="min-h-screen pansy-bg flex flex-col">
      {/* Header */}
      <div className={`bg-gradient-to-r ${sideColors.primary} backdrop-blur-lg border-b border-purple-500/30 p-4 shadow-lg sticky top-0 z-10`}>
        <div className="max-w-2xl mx-auto flex justify-between items-center">
          <div className="flex items-center gap-3">
            <span className="text-4xl">🌸</span>
            <div>
              <h1 className="text-2xl font-bold">SOL</h1>
              <p className="text-white/80 text-xs">Messages for you 💜</p>
            </div>
          </div>
          <div className="flex items-center gap-2">
            <button
              onClick={() => setShowCalendar(!showCalendar)}
              className="bg-white/10 hover:bg-white/20 p-2 rounded-full transition-all"
              title="Calendar"
            >
              <CalendarIcon className="w-5 h-5" />
            </button>
            <button
              onClick={handleLogout}
              className="bg-white/10 hover:bg-white/20 p-2 rounded-full transition-all"
              title="Logout"
            >
              <LogOut className="w-5 h-5" />
            </button>
          </div>
        </div>
      </div>

      {/* Calendar (collapsible) */}
      {showCalendar && (
        <div className="max-w-2xl mx-auto w-full p-4">
          <Calendar 
            posts={posts} 
            onDateSelect={handleDateSelect}
            selectedDate={selectedDate}
            onClearFilter={handleClearFilter}
          />
        </div>
      )}

      {/* Chat Messages */}
      <div className="flex-1 overflow-y-auto p-4 max-w-2xl mx-auto w-full">
        {loading ? (
          <div className="text-center text-purple-300 py-12">
            <div className="text-4xl mb-4 animate-pulse">🌸</div>
            <p>Loading memories...</p>
          </div>
        ) : filteredPosts.length === 0 ? (
          <div className="text-center text-purple-300 py-12">
            <div className="text-6xl mb-4">💐</div>
            <p className="text-xl">{selectedDate ? 'No messages on this day...' : 'No messages yet...'}</p>
            <p className="text-sm mt-2 opacity-70">
              {selectedDate ? 'Try selecting a different date' : 'Your garden is waiting to bloom!'}
            </p>
          </div>
        ) : (
          <div className="space-y-4 pb-4">
            {filteredPosts.map((post) => {
              const isFromCurrentUser = post.from === userSide;
              
              return (
                <div
                  key={post.id}
                  className={`flex ${isFromCurrentUser ? 'justify-end' : 'justify-start'} ${
                    isFromCurrentUser ? 'chat-bubble-his' : 'chat-bubble-hers'
                  }`}
                >
                  <div className={`max-w-[80%] sm:max-w-[70%] group`}>
                    {/* Message bubble */}
                    <div
                      className={`
                        ${isFromCurrentUser ? sideColors.bubble : 'bg-gradient-to-br from-purple-800 to-purple-900'}
                        rounded-2xl p-3 shadow-lg
                        ${isFromCurrentUser ? 'rounded-br-sm' : 'rounded-bl-sm'}
                      `}
                    >
                      {/* Photo */}
                      {post.type === 'photo' && post.content && (
                        <img 
                          src={post.content} 
                          alt="Memory" 
                          className="rounded-xl mb-2 w-full"
                        />
                      )}
                      
                      {/* Text message */}
                      {post.type === 'message' && (
                        <p className="text-white leading-relaxed">{post.content}</p>
                      )}

                      {/* Caption */}
                      {post.caption && (
                        <p className="text-white/90 text-sm mt-2">{post.caption}</p>
                      )}

                      {/* Spotify */}
                      {post.musicTrack && (
                        <div className="mt-3 bg-black/30 rounded-xl p-2 backdrop-blur-sm">
                          <div className="flex items-center gap-2 mb-2">
                            {post.musicTrack.image && (
                              <img
                                src={post.musicTrack.image}
                                alt={post.musicTrack.name}
                                className="w-10 h-10 rounded"
                              />
                            )}
                            <div className="flex-1 min-w-0">
                              <p className="font-semibold text-sm truncate text-white">
                                {post.musicTrack.name}
                              </p>
                              <p className="text-xs text-white/70 truncate">{post.musicTrack.artist}</p>
                            </div>
                            <a
                              href={post.musicTrack.link || `https://www.deezer.com/track/${post.musicTrack.id}`}
                              target="_blank"
                              rel="noopener noreferrer"
                              className="text-purple-400 hover:text-purple-300"
                            >
                              <ExternalLink className="w-4 h-4" />
                            </a>
                          </div>
                          {post.musicTrack.previewUrl && (
                            <audio
                              controls
                              src={post.musicTrack.previewUrl}
                              className="w-full rounded"
                              style={{ height: '32px' }}
                            >
                              Your browser does not support audio playback.
                            </audio>
                          )}
                        </div>
                      )}
                    </div>

                    {/* Timestamp & Delete */}
                    <div className={`flex items-center gap-2 mt-1 px-2 ${isFromCurrentUser ? 'justify-end' : 'justify-start'}`}>
                      <span className="text-xs text-purple-400">
                        {new Date(post.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                      </span>
                      <button
                        onClick={() => handleDeletePost(post.id)}
                        className="opacity-0 group-hover:opacity-100 text-red-400 hover:text-red-300 transition-all"
                      >
                        <Trash2 className="w-3 h-3" />
                      </button>
                    </div>
                  </div>
                </div>
              );
            })}
            <div ref={messagesEndRef} />
          </div>
        )}
      </div>

      {/* Bottom input area */}
      <div className="sticky bottom-0 bg-gradient-to-t from-black via-purple-950/80 to-transparent backdrop-blur-lg border-t border-purple-500/30 p-4">
        <div className="max-w-2xl mx-auto">
          {!showUpload ? (
            <button
              onClick={() => setShowUpload(true)}
              className={`w-full bg-gradient-to-r ${sideColors.primary} text-white py-4 rounded-2xl font-semibold shadow-lg hover:shadow-xl transition-all flex items-center justify-center gap-2`}
            >
              <Camera className="w-5 h-5" />
              Leave a Memory 💌
            </button>
          ) : (
            <div className="glass rounded-2xl p-4 border border-purple-500/30">
              <div className="flex justify-between items-center mb-3">
                <h3 className="font-bold text-purple-300">Create Memory</h3>
                <button onClick={() => setShowUpload(false)} className="text-purple-400 hover:text-purple-300">
                  <X className="w-5 h-5" />
                </button>
              </div>

              <div className="flex gap-2 mb-3">
                <button
                  onClick={() => setNewPost({ ...newPost, type: 'photo', content: '' })}
                  className={`flex-1 py-2 rounded-lg font-semibold transition-all ${
                    newPost.type === 'photo'
                      ? 'bg-purple-600 text-white'
                      : 'bg-purple-900/30 text-purple-300'
                  }`}
                >
                  <Camera className="w-4 h-4 inline mr-2" />
                  Photo
                </button>
                <button
                  onClick={() => setNewPost({ ...newPost, type: 'message', content: '' })}
                  className={`flex-1 py-2 rounded-lg font-semibold transition-all ${
                    newPost.type === 'message'
                      ? 'bg-pink-600 text-white'
                      : 'bg-purple-900/30 text-purple-300'
                  }`}
                >
                  <MessageSquare className="w-4 h-4 inline mr-2" />
                  Message
                </button>
              </div>

              {newPost.type === 'photo' && (
                <div className="mb-3">
                  <label className="block w-full border-2 border-dashed border-purple-500/30 rounded-xl p-6 text-center cursor-pointer hover:border-purple-500 transition-all">
                    <input
                      type="file"
                      accept="image/*"
                      onChange={handleImageUpload}
                      className="hidden"
                    />
                    {newPost.content ? (
                      <img src={newPost.content} alt="Preview" className="max-h-48 mx-auto rounded-lg" />
                    ) : (
                      <div>
                        <ImageIcon className="w-10 h-10 mx-auto mb-2 text-purple-400" />
                        <p className="text-purple-300 text-sm">Tap to choose photo</p>
                      </div>
                    )}
                  </label>
                </div>
              )}

              {newPost.type === 'message' && (
                <textarea
                  value={newPost.content}
                  onChange={(e) => setNewPost({ ...newPost, content: e.target.value })}
                  placeholder="Write something sweet..."
                  className="w-full bg-purple-950/50 border border-purple-500/30 rounded-xl p-3 mb-3 h-24 focus:border-purple-500 focus:outline-none resize-none text-white placeholder-purple-400"
                />
              )}

              <input
                type="text"
                value={newPost.caption}
                onChange={(e) => setNewPost({ ...newPost, caption: e.target.value })}
                placeholder="Add a caption (optional) 💭"
                className="w-full bg-purple-950/50 border border-purple-500/30 rounded-xl p-3 mb-3 focus:border-purple-500 focus:outline-none text-white placeholder-purple-400"
              />

              <div className="mb-3">
                {newPost.musicTrack ? (
                    <div className="flex items-center gap-2 p-2 bg-green-950/30 border border-green-500/30 rounded-xl">
                      {newPost.musicTrack.image && (
                        <img
                          src={newPost.musicTrack.image}
                          alt={newPost.musicTrack.name}
                          className="w-10 h-10 rounded"
                        />
                      )}
                      <div className="flex-1 min-w-0">
                        <p className="font-semibold text-sm truncate text-white">{newPost.musicTrack.name}</p>
                        <p className="text-xs text-purple-300 truncate">{newPost.musicTrack.artist}</p>
                      </div>
                      <button
                        onClick={() => setNewPost({ ...newPost, musicTrack: null })}
                        className="text-red-400 hover:text-red-300"
                      >
                        <X className="w-4 h-4" />
                      </button>
                    </div>
                  ) : (
                    <button
                      onClick={() => setShowMusicSearch(true)}
                      className="w-full bg-gradient-to-r from-purple-600 to-pink-600 hover:from-purple-700 hover:to-pink-700 text-white py-2 rounded-xl font-semibold transition-all flex items-center justify-center gap-2"
                    >
                      <Music className="w-4 h-4" />
                      Add Song 🎵
                    </button>
                  )}
                </div>

              <button
                onClick={handleSubmitPost}
                className={`w-full bg-gradient-to-r ${sideColors.primary} text-white py-3 rounded-xl font-semibold hover:shadow-lg transition-all flex items-center justify-center gap-2`}
              >
                <Send className="w-4 h-4" />
                Send to Garden 🌸
              </button>
            </div>
          )}
        </div>
      </div>

      {/* Music Search Modal */}
      {showMusicSearch && (
        <DeezerSearch
          onSelectTrack={handleSelectMusicTrack}
          onClose={() => setShowMusicSearch(false)}
        />
      )}
    </div>
  );
}
