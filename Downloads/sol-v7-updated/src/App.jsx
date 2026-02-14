import React, { useState, useEffect, useRef } from 'react';
import { Camera, MessageSquare, Image as ImageIcon, X, LogOut, Trash2, Music, Calendar as CalendarIcon, Send, User, Search } from 'lucide-react';
import Calendar from './components/Calendar';
import { searchTracks, extractTrackId } from './spotifyApi';
import { APP_CONFIG } from './config.loader';
import './App.css';

export default function SOL() {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [userSide, setUserSide] = useState(null);
  const [password, setPassword] = useState('');
  const [posts, setPosts] = useState([]);
  const [showUpload, setShowUpload] = useState(false);
  const [newPost, setNewPost] = useState({ type: 'photo', content: '', caption: '', spotifyTrack: null });
  const [loading, setLoading] = useState(true);
  const [showSpotifySearch, setShowSpotifySearch] = useState(false);
  const [spotifyQuery, setSpotifyQuery] = useState('');
  const [spotifyResults, setSpotifyResults] = useState([]);
  const [spotifyLoading, setSpotifyLoading] = useState(false);
  const [spotifyConnected, setSpotifyConnected] = useState(false);
  const [recentSearches, setRecentSearches] = useState([]);
  const [showCalendar, setShowCalendar] = useState(false);
  const [selectedDate, setSelectedDate] = useState(null);
  const [hisProfile, setHisProfile] = useState(null);
  const [hersProfile, setHersProfile] = useState(null);
  const [expandedImage, setExpandedImage] = useState(null);
  const [showCamera, setShowCamera] = useState(false);
  const [stream, setStream] = useState(null);
  const messagesEndRef = useRef(null);
  const videoRef = useRef(null);
  const canvasRef = useRef(null);

  const isArtifact = typeof window !== 'undefined' && window.storage;

  const partnerName = userSide === 'his' ? APP_CONFIG.users.hers : APP_CONFIG.users.his;
  const userName = userSide === 'his' ? APP_CONFIG.users.his : APP_CONFIG.users.hers;

  // Load profiles
  useEffect(() => {
    const savedHisProfile = localStorage.getItem('sol_his_profile');
    const savedHersProfile = localStorage.getItem('sol_hers_profile');
    if (savedHisProfile) setHisProfile(savedHisProfile);
    if (savedHersProfile) setHersProfile(savedHersProfile);

    // Load recent searches
    const savedSearches = localStorage.getItem('sol_recent_searches');
    if (savedSearches) setRecentSearches(JSON.parse(savedSearches));
  }, []);

  useEffect(() => {
    if (isAuthenticated) {
      loadPosts();
    }
  }, [isAuthenticated]);

  useEffect(() => {
    scrollToBottom();
  }, [posts]);

  // Cleanup camera stream when component unmounts
  useEffect(() => {
    return () => {
      if (stream) {
        stream.getTracks().forEach(track => track.stop());
      }
    };
  }, [stream]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const loadPosts = async () => {
    try {
      setLoading(true);
      let allPosts = [];
      
      // Try to load from API first (works across devices)
      try {
        const response = await fetch('/api/posts/get');
        if (response.ok) {
          const data = await response.json();
          const hisPosts = data.his || [];
          const hersPosts = data.hers || [];
          allPosts = [...hisPosts, ...hersPosts];
          console.log('Loaded from API:', allPosts.length, 'posts');
        } else {
          throw new Error('API not available');
        }
      } catch (apiError) {
        console.log('API not available, using localStorage:', apiError.message);
        
        // Fallback to localStorage (device-specific)
        const hisStored = localStorage.getItem(`sol_posts_his`);
        const hersStored = localStorage.getItem(`sol_posts_hers`);
        
        const hisPosts = hisStored ? JSON.parse(hisStored) : [];
        const hersPosts = hersStored ? JSON.parse(hersStored) : [];
        
        allPosts = [...hisPosts, ...hersPosts];
        console.log('Loaded from localStorage:', allPosts.length, 'posts');
      }
      
      allPosts.sort((a, b) => new Date(a.timestamp) - new Date(b.timestamp));
      setPosts(allPosts);
    } catch (error) {
      console.log('Error loading posts:', error);
      setPosts([]);
    } finally {
      setLoading(false);
    }
  };

  const handleLogin = (e) => {
    e.preventDefault();
    if (password === APP_CONFIG.passwords.his) {
      setUserSide('his');
      setIsAuthenticated(true);
    } else if (password === APP_CONFIG.passwords.hers) {
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
    if (stream) {
      stream.getTracks().forEach(track => track.stop());
      setStream(null);
    }
  };

  const startCamera = async () => {
    try {
      const mediaStream = await navigator.mediaDevices.getUserMedia({ 
        video: { facingMode: 'environment' }, // Use back camera on mobile
        audio: false 
      });
      setStream(mediaStream);
      setShowCamera(true);
      
      // Wait for next frame to ensure video element is rendered
      setTimeout(() => {
        if (videoRef.current) {
          videoRef.current.srcObject = mediaStream;
        }
      }, 100);
    } catch (error) {
      console.error('Camera access error:', error);
      alert('Unable to access camera. Please check permissions.');
    }
  };

  const capturePhoto = () => {
    if (!videoRef.current || !canvasRef.current) return;
    
    const video = videoRef.current;
    const canvas = canvasRef.current;
    const context = canvas.getContext('2d');
    
    // Set canvas size to video size
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;
    
    // Draw video frame to canvas
    context.drawImage(video, 0, 0, canvas.width, canvas.height);
    
    // Convert to data URL
    const imageData = canvas.toDataURL('image/jpeg', 0.8);
    setNewPost({ ...newPost, content: imageData });
    
    // Stop camera
    stopCamera();
  };

  const stopCamera = () => {
    if (stream) {
      stream.getTracks().forEach(track => track.stop());
      setStream(null);
    }
    setShowCamera(false);
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

  const handleProfileUpload = (e) => {
    const file = e.target.files[0];
    if (file) {
      const reader = new FileReader();
      reader.onloadend = () => {
        // Each user sets their OWN profile
        if (userSide === 'his') {
          setHisProfile(reader.result);
          localStorage.setItem('sol_his_profile', reader.result);
        } else {
          setHersProfile(reader.result);
          localStorage.setItem('sol_hers_profile', reader.result);
        }
      };
      reader.readAsDataURL(file);
    }
  };

  const addRecentSearch = (query) => {
    const updated = [query, ...recentSearches.filter(q => q !== query)].slice(0, 5);
    setRecentSearches(updated);
    localStorage.setItem('sol_recent_searches', JSON.stringify(updated));
  };

  const searchSpotify = async () => {
    if (!spotifyQuery.trim()) return;
    
    // Check if it's a Spotify link
    const trackId = extractTrackId(spotifyQuery);
    if (trackId) {
      const track = {
        id: trackId,
        name: 'Spotify Track',
        artist: 'Loading...',
        album: '',
        image: ''
      };
      setNewPost({ ...newPost, spotifyTrack: track });
      setShowSpotifySearch(false);
      setSpotifyQuery('');
      setSpotifyResults([]);
      setSpotifyConnected(true); // Mark as connected
      return;
    }

    // Search by query using real Spotify API
    try {
      setSpotifyLoading(true);
      // Add to recent searches
      addRecentSearch(spotifyQuery);
      
      // Call Spotify search
      const results = await searchTracks(spotifyQuery);
      setSpotifyResults(results);
      
      // If we got results, mark as connected
      if (results && results.length > 0) {
        setSpotifyConnected(true);
      }
    } catch (error) {
      console.log('Spotify search error:', error);
      // Show empty results on error
      setSpotifyResults([]);
      setSpotifyConnected(false);
    } finally {
      setSpotifyLoading(false);
    }
  };

  const checkSpotifyConnection = async () => {
    try {
      const response = await fetch('/api/spotify/check');
      const data = await response.json();
      setSpotifyConnected(data.connected);
      
      if (data.connected) {
        alert('✅ Spotify is connected!');
      } else {
        // Redirect to Spotify login
        window.location.href = '/api/spotify/login';
      }
    } catch (error) {
      console.log('Spotify check error:', error);
      setSpotifyConnected(false);
      // Redirect to Spotify login anyway
      window.location.href = '/api/spotify/login';
    }
  };

  const handleSelectSpotifyTrack = (track) => {
    setNewPost({ ...newPost, spotifyTrack: track });
    setShowSpotifySearch(false);
    setSpotifyQuery('');
    setSpotifyResults([]);
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
      spotifyTrack: newPost.spotifyTrack,
      timestamp: new Date().toISOString(),
      from: userSide
    };

    try {
      let existingPosts = [];
      
      // Try to save to API first (syncs across devices)
      try {
        // Get existing posts from localStorage for this user
        const stored = localStorage.getItem(`sol_posts_${userSide}`);
        if (stored) {
          existingPosts = JSON.parse(stored);
        }
        
        const updatedPosts = [post, ...existingPosts];
        
        // Save to API
        const response = await fetch('/api/posts/save', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            userSide: userSide,
            posts: updatedPosts
          })
        });
        
        if (response.ok) {
          console.log('Saved to API successfully');
          // Also save to localStorage as backup
          localStorage.setItem(`sol_posts_${userSide}`, JSON.stringify(updatedPosts));
        } else {
          throw new Error('API save failed');
        }
      } catch (apiError) {
        console.log('API not available, using localStorage only:', apiError.message);
        
        // Fallback: Save to localStorage only
        const stored = localStorage.getItem(`sol_posts_${userSide}`);
        if (stored) {
          existingPosts = JSON.parse(stored);
        }
        
        const updatedPosts = [post, ...existingPosts];
        localStorage.setItem(`sol_posts_${userSide}`, JSON.stringify(updatedPosts));
      }

      // Reload posts
      await loadPosts();
      
      // Reset form
      setNewPost({ type: 'photo', content: '', caption: '', spotifyTrack: null });
      setShowUpload(false);
    } catch (error) {
      console.error('Error saving post:', error);
      alert('Failed to save post. Please try again.');
    }
  };

  const handleDeletePost = async (postId) => {
    if (!confirm('Delete this memory? 🥺')) return;

    try {
      // Try API first
      try {
        const stored = localStorage.getItem(`sol_posts_${userSide}`);
        if (stored) {
          const existingPosts = JSON.parse(stored);
          const updatedPosts = existingPosts.filter(p => p.id !== postId);
          
          // Save to API
          const response = await fetch('/api/posts/save', {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              userSide: userSide,
              posts: updatedPosts
            })
          });
          
          if (response.ok) {
            localStorage.setItem(`sol_posts_${userSide}`, JSON.stringify(updatedPosts));
          } else {
            throw new Error('API delete failed');
          }
        }
      } catch (apiError) {
        console.log('API not available, using localStorage:', apiError.message);
        
        // Fallback to localStorage
        const stored = localStorage.getItem(`sol_posts_${userSide}`);
        if (stored) {
          const existingPosts = JSON.parse(stored);
          const updatedPosts = existingPosts.filter(p => p.id !== postId);
          localStorage.setItem(`sol_posts_${userSide}`, JSON.stringify(updatedPosts));
        }
      }
      
      // Reload posts
      await loadPosts();
    } catch (error) {
      console.error('Error deleting post:', error);
    }
  };

  const formatTime = (timestamp) => {
    const date = new Date(timestamp);
    const now = new Date();
    const diff = now - date;
    const minutes = Math.floor(diff / 60000);
    const hours = Math.floor(diff / 3600000);
    const days = Math.floor(diff / 86400000);

    if (minutes < 1) return 'Just now';
    if (minutes < 60) return `${minutes}m ago`;
    if (hours < 24) return `${hours}h ago`;
    if (days < 7) return `${days}d ago`;
    
    return date.toLocaleDateString('en-US', { month: 'short', day: 'numeric' });
  };

  if (!isAuthenticated) {
    return (
      <div className="login-screen">
        <div className="login-container">
          <div className="login-header">
            <img src="/pansy.png" alt="Pansy" className="login-logo" />
            <h1 className="login-title">SOL</h1>
            <p className="login-subtitle">Our Little Garden 🌸</p>
          </div>
          <form onSubmit={handleLogin} className="login-form">
            <input
              type="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="Enter your password..."
              className="login-input"
              autoFocus
            />
            <button type="submit" className="login-button">
              Enter Garden 💜
            </button>
          </form>
        </div>
      </div>
    );
  }

  return (
    <div className="app-container">
      {/* Calendar Overlay */}
      {showCalendar && <div className="calendar-overlay" onClick={() => setShowCalendar(false)} />}
      
      {/* Calendar Sidebar */}
      <div className={`calendar-sidebar ${showCalendar ? 'open' : ''}`}>
        <div className="calendar-sidebar-content">
          <div className="calendar-sidebar-header">
            <h2>Our Calendar 📅</h2>
            <button onClick={() => setShowCalendar(false)} className="calendar-close">
              <X size={24} />
            </button>
          </div>
          <Calendar 
            posts={posts} 
            onDateSelect={(date) => {
              setSelectedDate(date);
              setShowCalendar(false);
            }}
          />
        </div>
      </div>

      {/* Header */}
      <div className="chat-header">
        <button onClick={handleLogout} className="back-button">←</button>
        
        <div className="chat-header-info">
          <div className="partner-avatar-display">
            {(userSide === 'his' ? hersProfile : hisProfile) ? (
              <img 
                src={userSide === 'his' ? hersProfile : hisProfile} 
                alt={partnerName}
                className="header-avatar"
              />
            ) : (
              <div className="header-avatar-placeholder">
                {partnerName[0]}
              </div>
            )}
          </div>
          
          <div className="header-text">
            <div className="header-name">{partnerName} 💜</div>
            <div className="header-status">Our Little Garden</div>
          </div>
        </div>

        <div className="header-actions">
          <button 
            onClick={checkSpotifyConnection}
            className={`header-icon-button ${spotifyConnected ? 'spotify-connected' : 'spotify-disconnected'}`}
            title={spotifyConnected ? "Spotify Connected ✓" : "Connect to Spotify"}
          >
            <Music className="header-icon" />
          </button>
          
          <button onClick={() => setShowCalendar(true)} className="header-icon-button">
            <CalendarIcon className="header-icon" />
          </button>
          
          <label className="header-icon-button">
            <input
              type="file"
              accept="image/*"
              onChange={handleProfileUpload}
              className="hidden"
            />
            <User className="header-icon" />
          </label>
        </div>
      </div>

      {/* Messages Area */}
      <div className="messages-area">
        {loading ? (
          <div className="messages-empty">Loading memories... 🌸</div>
        ) : posts.length === 0 ? (
          <div className="messages-empty">
            No memories yet! 💭<br />
            Start your garden by leaving your first memory below 🌱
          </div>
        ) : (
          posts.map((post) => {
            const isFromCurrentUser = post.from === userSide;
            const senderProfile = post.from === 'his' ? hisProfile : hersProfile;
            const senderName = post.from === 'his' ? 'K' : 'P';

            return (
              <div 
                key={post.id} 
                className={`message-row ${isFromCurrentUser ? 'message-sent' : 'message-received'}`}
              >
                {!isFromCurrentUser && (
                  <div className="message-avatar-container">
                    {senderProfile ? (
                      <img src={senderProfile} alt="" className="message-avatar" />
                    ) : (
                      <div className="message-avatar-placeholder">{senderName}</div>
                    )}
                  </div>
                )}
                
                <div className="message-bubble-container">
                  <div className={`message-bubble ${isFromCurrentUser ? 'bubble-sent' : 'bubble-received'}`}>
                    {post.type === 'photo' && (
                      <div>
                        <img 
                          src={post.content} 
                          alt="Memory" 
                          className="message-photo"
                          onClick={() => setExpandedImage(post.id)}
                        />
                        {post.caption && (
                          <p className="message-caption-text">{post.caption}</p>
                        )}
                      </div>
                    )}
                    
                    {post.type === 'message' && (
                      <p className="message-text-content">{post.content}</p>
                    )}

                    {post.spotifyTrack && (
                      <div className="spotify-container">
                        <div className="spotify-info">
                          {post.spotifyTrack.image && (
                            <img src={post.spotifyTrack.image} alt="" className="spotify-image" />
                          )}
                          <div className="spotify-text">
                            <p className="spotify-name">{post.spotifyTrack.name}</p>
                            <p className="spotify-artist">{post.spotifyTrack.artist}</p>
                          </div>
                        </div>
                        <div className="spotify-player">
                          <iframe
                            src={`https://open.spotify.com/embed/track/${post.spotifyTrack.id}?utm_source=generator&theme=0`}
                            width="100%"
                            height="80"
                            frameBorder="0"
                            allow="autoplay; clipboard-write; encrypted-media; fullscreen; picture-in-picture"
                            loading="lazy"
                            title="Spotify Player"
                          ></iframe>
                        </div>
                      </div>
                    )}
                  </div>
                  
                  <div className={`message-meta ${isFromCurrentUser ? 'meta-sent' : 'meta-received'}`}>
                    <span className="message-time">{formatTime(post.timestamp)}</span>
                    {userSide === 'his' && (
                      <button
                        onClick={() => handleDeletePost(post.id)}
                        className="message-delete"
                      >
                        <Trash2 className="delete-icon" />
                      </button>
                    )}
                  </div>
                </div>
              </div>
            );
          })
        )}
        <div ref={messagesEndRef} />
      </div>

      {/* Input Area - Messenger Style */}
      <div className="input-bar">
        {!showUpload ? (
          <div className="messenger-input-container">
            <button 
              onClick={() => {
                setNewPost({ ...newPost, type: 'photo', content: '' });
                setShowUpload(true);
              }}
              className="messenger-icon-button messenger-purple"
              title="Add photo"
            >
              <ImageIcon size={24} />
            </button>
            
            <button 
              onClick={() => {
                setNewPost({ ...newPost, type: 'photo', content: '' });
                setShowUpload(true);
                setTimeout(() => startCamera(), 100);
              }}
              className="messenger-icon-button messenger-purple"
              title="Take photo"
            >
              <Camera size={24} />
            </button>
            
            <button 
              onClick={() => setShowSpotifySearch(true)}
              className="messenger-icon-button messenger-spotify"
              title="Add song"
            >
              <Music size={20} />
            </button>
            
            <input
              type="text"
              placeholder="Send a message..."
              className="messenger-text-input"
              onFocus={() => {
                setNewPost({ ...newPost, type: 'message', content: '' });
                setShowUpload(true);
              }}
              readOnly
            />
          </div>
        ) : (
          <div className="compose-container">
            <div className="compose-type-selector">
              <button
                onClick={() => {
                  setNewPost({ ...newPost, type: 'photo', content: '' });
                  stopCamera();
                }}
                className={`type-button ${newPost.type === 'photo' ? 'type-active' : ''}`}
              >
                <Camera className="type-icon" />
                Photo
              </button>
              <button
                onClick={() => {
                  setNewPost({ ...newPost, type: 'message', content: '' });
                  stopCamera();
                }}
                className={`type-button ${newPost.type === 'message' ? 'type-active' : ''}`}
              >
                <MessageSquare className="type-icon" />
                Message
              </button>
              <button
                onClick={() => {
                  setShowUpload(false);
                  stopCamera();
                }}
                className="type-button type-close"
              >
                <X className="type-icon" />
                Close
              </button>
            </div>

            {newPost.type === 'photo' && !showCamera && !newPost.content && (
              <label className="photo-upload-area">
                <input
                  type="file"
                  accept="image/*"
                  capture="environment"
                  onChange={handleImageUpload}
                  className="hidden"
                />
                <div className="photo-upload-placeholder">
                  <Camera className="upload-icon" />
                  <p>Tap to take or choose photo</p>
                </div>
              </label>
            )}

            {showCamera && (
              <div className="camera-view">
                <video 
                  ref={videoRef} 
                  autoPlay 
                  playsInline
                  className="camera-video"
                />
                <canvas ref={canvasRef} className="hidden" />
                <div className="camera-controls">
                  <button onClick={stopCamera} className="camera-control-button cancel">
                    <X size={24} />
                  </button>
                  <button onClick={capturePhoto} className="camera-control-button capture">
                    <div className="capture-ring" />
                  </button>
                </div>
              </div>
            )}

            {newPost.type === 'photo' && newPost.content && !showCamera && (
              <div className="photo-preview-container">
                <img src={newPost.content} alt="Preview" className="photo-preview" />
                <button 
                  onClick={() => setNewPost({ ...newPost, content: '' })}
                  className="photo-remove"
                >
                  <X size={20} />
                </button>
              </div>
            )}

            {newPost.type === 'message' && (
              <textarea
                value={newPost.content}
                onChange={(e) => setNewPost({ ...newPost, content: e.target.value })}
                placeholder="Write something sweet..."
                className="compose-textarea"
              />
            )}

            {newPost.type === 'photo' && newPost.content && !showCamera && (
              <input
                type="text"
                value={newPost.caption}
                onChange={(e) => setNewPost({ ...newPost, caption: e.target.value })}
                placeholder="Add a caption (optional) 💭"
                className="compose-input"
              />
            )}

            {!showCamera && (
              <>
                <div className="spotify-section">
                  {newPost.spotifyTrack ? (
                    <div className="spotify-selected">
                      {newPost.spotifyTrack.image && (
                        <img src={newPost.spotifyTrack.image} alt="" className="spotify-selected-image" />
                      )}
                      <div className="spotify-selected-text">
                        <p className="spotify-selected-name">{newPost.spotifyTrack.name}</p>
                        <p className="spotify-selected-artist">{newPost.spotifyTrack.artist}</p>
                      </div>
                      <button
                        onClick={() => setNewPost({ ...newPost, spotifyTrack: null })}
                        className="spotify-remove"
                      >
                        <X />
                      </button>
                    </div>
                  ) : (
                    <button
                      onClick={() => setShowSpotifySearch(true)}
                      className="spotify-add-button"
                    >
                      <svg className="spotify-logo" viewBox="0 0 24 24" fill="currentColor">
                        <path d="M12 0C5.4 0 0 5.4 0 12s5.4 12 12 12 12-5.4 12-12S18.66 0 12 0zm5.521 17.34c-.24.359-.66.48-1.021.24-2.82-1.74-6.36-2.101-10.561-1.141-.418.122-.779-.179-.899-.539-.12-.421.18-.78.54-.9 4.56-1.021 8.52-.6 11.64 1.32.42.18.479.659.301 1.02zm1.44-3.3c-.301.42-.841.6-1.262.3-3.239-1.98-8.159-2.58-11.939-1.38-.479.12-1.02-.12-1.14-.6-.12-.48.12-1.021.6-1.141C9.6 9.9 15 10.561 18.72 12.84c.361.181.54.78.241 1.2zm.12-3.36C15.24 8.4 8.82 8.16 5.16 9.301c-.6.179-1.2-.181-1.38-.721-.18-.601.18-1.2.72-1.381 4.26-1.26 11.28-1.02 15.721 1.621.539.3.719 1.02.419 1.56-.299.421-1.02.599-1.559.3z"/>
                      </svg>
                      Add Song 🎵
                    </button>
                  )}
                </div>

                <button onClick={handleSubmitPost} className="compose-submit">
                  <Send className="submit-icon" />
                  Send to Garden 🌸
                </button>
              </>
            )}
          </div>
        )}
      </div>

      {/* Spotify Search Modal */}
      {showSpotifySearch && (
        <div className="modal-overlay" onClick={() => setShowSpotifySearch(false)}>
          <div className="modal-content" onClick={(e) => e.stopPropagation()}>
            <div className="modal-header">
              <h3>Add Song</h3>
              <button onClick={() => setShowSpotifySearch(false)} className="modal-close">
                <X />
              </button>
            </div>
            
            <div className="search-bar">
              <Search className="search-icon" />
              <input
                type="text"
                value={spotifyQuery}
                onChange={(e) => setSpotifyQuery(e.target.value)}
                onKeyPress={(e) => e.key === 'Enter' && searchSpotify()}
                placeholder="Search song name or paste Spotify link..."
                className="search-input"
                autoFocus
              />
            </div>
            
            <button onClick={searchSpotify} className="search-button" disabled={spotifyLoading}>
              {spotifyLoading ? 'Searching...' : 'Search'}
            </button>

            {/* Recent Searches */}
            {recentSearches.length > 0 && spotifyResults.length === 0 && !spotifyLoading && (
              <div className="recent-searches">
                <h4 className="recent-title">Recent Searches</h4>
                {recentSearches.map((search, index) => (
                  <button
                    key={index}
                    onClick={() => {
                      setSpotifyQuery(search);
                    }}
                    className="recent-item"
                  >
                    <Search className="recent-icon" />
                    <span>{search}</span>
                  </button>
                ))}
              </div>
            )}

            {/* Search Results */}
            {spotifyLoading && (
              <div className="search-loading">Searching Spotify... 🎵</div>
            )}
            
            <div className="search-results">
              {spotifyResults.map((track) => (
                <button
                  key={track.id}
                  onClick={() => handleSelectSpotifyTrack(track)}
                  className="search-result-item"
                >
                  {track.image && (
                    <img src={track.image} alt="" className="result-image" />
                  )}
                  <div className="result-text">
                    <p className="result-name">{track.name}</p>
                    <p className="result-artist">{track.artist}</p>
                  </div>
                </button>
              ))}
            </div>
          </div>
        </div>
      )}

      {/* Expanded Image Modal */}
      {expandedImage && (
        <div className="image-modal" onClick={() => setExpandedImage(null)}>
          <img
            src={posts.find(p => p.id === expandedImage)?.content}
            alt="Expanded"
            className="image-modal-content"
            onClick={(e) => e.stopPropagation()}
          />
        </div>
      )}
    </div>
  );
}
