'use client';

import React, { useState, useEffect } from 'react';
import { Music, Image, Plus, X, User, Heart, Smile, Meh, Frown, Sun, Cloud, Zap, Sparkles, TrendingUp, MessageCircle, Calendar as CalendarIcon, Camera } from 'lucide-react';

export default function MoodBoard() {
  const [songs, setSongs] = useState([]);
  const [images, setImages] = useState([]);
  const [moods, setMoods] = useState({});
  const [showAddSong, setShowAddSong] = useState(false);
  const [showAddImage, setShowAddImage] = useState(false);
  const [currentUser, setCurrentUser] = useState('Kyaw');
  const [loading, setLoading] = useState(true);
  const [selectedDate, setSelectedDate] = useState(null);
  const [aiInsights, setAiInsights] = useState(null);
  const [showInsights, setShowInsights] = useState(false);
  const [aiLoading, setAiLoading] = useState(false);

  // Form states
  const [songTitle, setSongTitle] = useState('');
  const [songArtist, setSongArtist] = useState('');
  const [albumArt, setAlbumArt] = useState('');
  const [imageFile, setImageFile] = useState(null);
  const [imagePreview, setImagePreview] = useState('');

  useEffect(() => {
    loadData();
    // Poll for updates every 5 seconds to sync with other users
    const interval = setInterval(loadData, 5000);
    return () => clearInterval(interval);
  }, []);

  const loadData = async () => {
    try {
      const response = await fetch('/api/data');
      const data = await response.json();
      
      if (data.songs) setSongs(data.songs);
      if (data.images) setImages(data.images);
      if (data.moods) setMoods(data.moods);
    } catch (error) {
      console.error('Failed to load data:', error);
    }
    setLoading(false);
  };

  const saveSongs = async (newSongs) => {
    try {
      await fetch('/api/data', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ type: 'songs', data: newSongs }),
      });
    } catch (error) {
      console.error('Failed to save songs:', error);
    }
  };

  const saveImages = async (newImages) => {
    try {
      await fetch('/api/data', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ type: 'images', data: newImages }),
      });
    } catch (error) {
      console.error('Failed to save images:', error);
    }
  };

  const saveMoods = async (newMoods) => {
    try {
      await fetch('/api/data', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ type: 'calendar', data: newMoods }),
      });
    } catch (error) {
      console.error('Failed to save moods:', error);
    }
  };

  const addSong = async () => {
    const newSong = {
      id: Date.now(),
      title: songTitle,
      artist: songArtist,
      albumArt: albumArt,
      addedBy: currentUser,
      timestamp: new Date().toISOString()
    };
    
    const newSongs = [newSong, ...songs];
    setSongs(newSongs);
    await saveSongs(newSongs);
    
    setSongTitle('');
    setSongArtist('');
    setAlbumArt('');
    setShowAddSong(false);
  };

  const deleteSong = async (id) => {
    const newSongs = songs.filter(song => song.id !== id);
    setSongs(newSongs);
    await saveSongs(newSongs);
  };

  const handleImageSelect = (e) => {
    const file = e.target.files[0];
    if (file) {
      setImageFile(file);
      const reader = new FileReader();
      reader.onloadend = () => {
        setImagePreview(reader.result);
      };
      reader.readAsDataURL(file);
    }
  };

  const addImage = async () => {
    const newImage = {
      id: Date.now(),
      url: imagePreview,
      addedBy: currentUser,
      timestamp: new Date().toISOString()
    };
    
    const newImages = [newImage, ...images];
    setImages(newImages);
    await saveImages(newImages);
    
    setImageFile(null);
    setImagePreview('');
    setShowAddImage(false);
  };

  const deleteImage = async (id) => {
    const newImages = images.filter(img => img.id !== id);
    setImages(newImages);
    await saveImages(newImages);
  };

  const setMood = async (date, moodData) => {
    const newMoods = { ...moods, [date]: moodData };
    setMoods(newMoods);
    await saveMoods(newMoods);
  };

  // AI FEATURES

  const generateInsights = async () => {
    setAiLoading(true);
    setShowInsights(true);

    try {
      const moodData = prepareMoodAnalysis();
      const musicData = prepareMusicAnalysis();
      const imageCount = images.length;

      const response = await fetch("https://api.anthropic.com/v1/messages", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          model: "claude-sonnet-4-20250514",
          max_tokens: 1000,
          messages: [
            {
              role: "user",
              content: `You're analyzing a couple's mood board app. Here's their data:

MOOD CALENDAR:
${JSON.stringify(moodData, null, 2)}

SHARED MUSIC (${songs.length} songs):
${JSON.stringify(musicData, null, 2)}

SHARED IMAGES: ${imageCount} photos

Generate insights in this EXACT JSON format (no markdown, no preamble):
{
  "moodAnalysis": {
    "kyawTrends": "brief analysis of Kyaw's mood patterns",
    "panTrends": "brief analysis of Pan's mood patterns",
    "compatibility": "how well moods align",
    "suggestion": "one actionable suggestion"
  },
  "musicInsights": {
    "compatibility": "music taste compatibility score and brief analysis",
    "recommendation": "one song recommendation based on their tastes"
  },
  "conversationStarters": [
    "question 1",
    "question 2",
    "question 3"
  ],
  "weekSummary": "brief narrative summary of their week together"
}`
            }
          ]
        })
      });

      const data = await response.json();
      const content = data.content.find(c => c.type === 'text')?.text || '';
      
      const cleaned = content.replace(/```json|```/g, '').trim();
      const insights = JSON.parse(cleaned);
      
      setAiInsights(insights);
    } catch (error) {
      console.error('AI Error:', error);
      setAiInsights({
        error: "Couldn't generate insights right now. Try again later!"
      });
    }
    setAiLoading(false);
  };

  const prepareMoodAnalysis = () => {
    const moodCounts = { Kyaw: {}, Pan: {} };
    const recent = [];
    
    Object.entries(moods).forEach(([date, dayMood]) => {
      if (dayMood.Kyaw) {
        moodCounts.Kyaw[dayMood.Kyaw.label] = (moodCounts.Kyaw[dayMood.Kyaw.label] || 0) + 1;
        recent.push({ date, user: 'Kyaw', mood: dayMood.Kyaw.label });
      }
      if (dayMood.Pan) {
        moodCounts.Pan[dayMood.Pan.label] = (moodCounts.Pan[dayMood.Pan.label] || 0) + 1;
        recent.push({ date, user: 'Pan', mood: dayMood.Pan.label });
      }
    });

    return {
      totalDays: Object.keys(moods).length,
      moodCounts,
      recentMoods: recent.slice(-10)
    };
  };

  const prepareMusicAnalysis = () => {
    return songs.slice(0, 10).map(s => ({
      title: s.title,
      artist: s.artist,
      addedBy: s.addedBy
    }));
  };

  const moodEmojis = [
    { icon: Smile, label: 'happy', color: '#ffeb3b' },
    { icon: Heart, label: 'loved', color: '#ff6b9d' },
    { icon: Sun, label: 'energetic', color: '#ff9800' },
    { icon: Cloud, label: 'calm', color: '#03a9f4' },
    { icon: Frown, label: 'anxious', color: '#795548' },
    { icon: Meh, label: 'okay', color: '#9e9e9e' },
    { icon: Cloud, label: 'meh', color: '#b0bec5' },
    { icon: Frown, label: 'sad', color: '#2196f3' },
    { icon: Zap, label: 'stressed', color: '#9c27b0' }
  ];

  const getCurrentMonth = () => {
    const now = new Date();
    const year = now.getFullYear();
    const month = now.getMonth();
    const firstDay = new Date(year, month, 1).getDay();
    const daysInMonth = new Date(year, month + 1, 0).getDate();
    
    return { year, month, firstDay, daysInMonth };
  };

  const { year, month, firstDay, daysInMonth } = getCurrentMonth();
  const monthNames = ['January', 'February', 'March', 'April', 'May', 'June', 
                      'July', 'August', 'September', 'October', 'November', 'December'];

  const calendarDays = [];
  for (let i = 0; i < firstDay; i++) {
    calendarDays.push(null);
  }
  for (let day = 1; day <= daysInMonth; day++) {
    calendarDays.push(day);
  }

  if (loading) {
    return (
      <div className="loading">
        <Heart className="heart-pulse" size={48} />
        <p>Loading...</p>
      </div>
    );
  }

  return (
    <div className="container">
      <div className="header">
        <h1 className="title">Kyaw & Pan</h1>
        
        <div className="user-toggle">
          <button 
            className={`user-btn ${currentUser === 'Kyaw' ? 'active' : ''}`}
            onClick={() => setCurrentUser('Kyaw')}
          >
            <User size={18} />
            Kyaw
          </button>
          <button 
            className={`user-btn ${currentUser === 'Pan' ? 'active' : ''}`}
            onClick={() => setCurrentUser('Pan')}
          >
            <User size={18} />
            Pan
          </button>
        </div>

        <button 
          className="ai-button" 
          onClick={generateInsights}
          disabled={aiLoading || (songs.length === 0 && Object.keys(moods).length === 0)}
        >
          <Sparkles size={20} />
          {aiLoading ? 'Generating Insights...' : 'AI Insights'}
        </button>
      </div>
      
      <div className="t-layout">
        {/* Mood Calendar */}
        <div className="calendar-section">
          <div className="section-header">
            <Heart size={24} />
            Mood Calendar
          </div>
          <div className="month-title">{monthNames[month]} {year}</div>
          <div className="calendar-grid">
            {calendarDays.map((day, index) => {
              if (!day) return <div key={index} className="calendar-day empty" />;
              
              const dateKey = `${year}-${month + 1}-${day}`;
              const dayMoods = moods[dateKey] || {};
              
              return (
                <div 
                  key={index} 
                  className="calendar-day"
                  onClick={() => setSelectedDate(dateKey)}
                >
                  <div className="day-number">{day}</div>
                  <div className="day-moods">
                    {dayMoods.Kyaw && (
                      <div className="mood-indicator" style={{ color: dayMoods.Kyaw.color }}>
                        {React.createElement(dayMoods.Kyaw.icon, { size: 18 })}
                      </div>
                    )}
                    {dayMoods.Pan && (
                      <div className="mood-indicator" style={{ color: dayMoods.Pan.color }}>
                        {React.createElement(dayMoods.Pan.icon, { size: 18 })}
                      </div>
                    )}
                  </div>
                </div>
              );
            })}
          </div>
        </div>
        
        {/* Music */}
        <div className="music-section">
          <div className="section-header">
            <Music size={24} />
            Music
          </div>
          <button className="add-button" onClick={() => setShowAddSong(true)}>
            <Plus size={18} />
            Add Song
          </button>
          <div className="section-content">
            {songs.length === 0 ? (
              <div className="empty-state">No songs yet</div>
            ) : (
              songs.map(song => (
                <div key={song.id} className="song-item">
                  <div className="song-album-art">
                    {song.albumArt ? (
                      <img src={song.albumArt} alt={song.title} />
                    ) : (
                      <Music size={32} color="#ff6b9d" />
                    )}
                  </div>
                  <div className="song-details">
                    <div className="song-header">
                      <div className="song-meta">
                        <User size={14} />
                        {song.addedBy}
                      </div>
                      <button onClick={() => deleteSong(song.id)} className="delete-btn">
                        <X size={16} />
                      </button>
                    </div>
                    <div className="song-title">{song.title}</div>
                    <div className="song-artist">{song.artist}</div>
                  </div>
                </div>
              ))
            )}
          </div>
        </div>
        
        {/* Images */}
        <div className="images-section">
          <div className="section-header">
            <Image size={24} />
            Shared Images
          </div>
          <button className="add-button" onClick={() => setShowAddImage(true)}>
            <Plus size={18} />
            Add Image
          </button>
          <div className="section-content">
            {images.length === 0 ? (
              <div className="empty-state">No images yet</div>
            ) : (
              <div className="image-grid">
                {images.map(img => (
                  <div key={img.id} className="image-item">
                    <img src={img.url} alt="Shared moment" />
                    <div className="image-overlay">
                      <div className="image-meta">
                        <User size={12} />
                        {img.addedBy}
                      </div>
                      <button onClick={() => deleteImage(img.id)} className="delete-btn image-delete">
                        <X size={16} />
                      </button>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </div>
        </div>
      </div>
      
      {/* Mood Selector Modal */}
      {selectedDate && (
        <>
          <div className="modal-overlay" onClick={() => setSelectedDate(null)} />
          <div className="mood-selector">
            <h3>How are you feeling?</h3>
            <div className="mood-options">
              {moodEmojis.map(mood => {
                const isSelected = moods[selectedDate]?.[currentUser]?.label === mood.label;
                return (
                  <div 
                    key={mood.label}
                    className={`mood-option ${isSelected ? 'selected' : ''}`}
                    onClick={() => {
                      setMood(selectedDate, {
                        ...moods[selectedDate],
                        [currentUser]: mood
                      });
                    }}
                  >
                    {React.createElement(mood.icon, { size: 32, color: mood.color })}
                    <div className="mood-label">{mood.label}</div>
                  </div>
                );
              })}
            </div>
            <button className="close-selector" onClick={() => setSelectedDate(null)}>
              Close
            </button>
          </div>
        </>
      )}
      
      {/* Add Song Modal */}
      {showAddSong && (
        <>
          <div className="modal-overlay" onClick={() => setShowAddSong(false)} />
          <div style={{ position: 'fixed', top: '50%', left: '50%', transform: 'translate(-50%, -50%)', zIndex: 1000 }}>
            <div className="add-form">
              <h3>Add a Song</h3>
              <input
                type="text"
                placeholder="Song title"
                value={songTitle}
                onChange={(e) => setSongTitle(e.target.value)}
                className="form-input"
              />
              <input
                type="text"
                placeholder="Artist"
                value={songArtist}
                onChange={(e) => setSongArtist(e.target.value)}
                className="form-input"
              />
              <input
                type="text"
                placeholder="Album art URL (optional)"
                value={albumArt}
                onChange={(e) => setAlbumArt(e.target.value)}
                className="form-input"
              />
              <div className="form-buttons">
                <button className="cancel-btn" onClick={() => setShowAddSong(false)}>
                  Cancel
                </button>
                <button 
                  onClick={addSong}
                  disabled={!songTitle || !songArtist}
                  className="submit-btn"
                >
                  Add
                </button>
              </div>
            </div>
          </div>
        </>
      )}
      
      {/* Add Image Modal */}
      {showAddImage && (
        <>
          <div className="modal-overlay" onClick={() => setShowAddImage(false)} />
          <div style={{ position: 'fixed', top: '50%', left: '50%', transform: 'translate(-50%, -50%)', zIndex: 1000 }}>
            <div className="add-form">
              <h3>Add an Image</h3>
              {imagePreview ? (
                <div className="image-preview-container">
                  <img src={imagePreview} alt="Preview" className="image-preview" />
                </div>
              ) : (
                <div className="file-upload-container">
                  <label className="file-upload-label">
                    <Camera size={48} color="#ff6b9d" />
                    <div className="file-upload-text">
                      <strong>Choose from gallery or camera</strong><br/>
                      Tap to select
                    </div>
                    <input
                      type="file"
                      accept="image/*"
                      capture="environment"
                      onChange={handleImageSelect}
                      className="file-upload-input"
                    />
                  </label>
                </div>
              )}
              <div className="form-buttons">
                <button className="cancel-btn" onClick={() => {
                  setShowAddImage(false);
                  setImagePreview('');
                  setImageFile(null);
                }}>
                  Cancel
                </button>
                <button 
                  onClick={addImage}
                  disabled={!imagePreview}
                  className="submit-btn"
                >
                  Add
                </button>
              </div>
            </div>
          </div>
        </>
      )}

      {/* AI Insights Panel */}
      {showInsights && (
        <>
          <div className="modal-overlay" onClick={() => setShowInsights(false)} />
          <div className="insights-panel">
            <h2>
              <Sparkles size={32} />
              AI Insights
            </h2>

            {aiLoading ? (
              <div className="loading-spinner">
                <Heart className="heart-pulse" size={48} />
                <p>Analyzing your connection...</p>
              </div>
            ) : aiInsights?.error ? (
              <div className="insight-section">
                <p>{aiInsights.error}</p>
              </div>
            ) : aiInsights ? (
              <>
                <div className="insight-section">
                  <h3>
                    <TrendingUp size={20} />
                    Mood Analysis
                  </h3>
                  <p><strong>Kyaw&apos;s patterns:</strong> {aiInsights.moodAnalysis?.kyawTrends}</p>
                  <p><strong>Pan&apos;s patterns:</strong> {aiInsights.moodAnalysis?.panTrends}</p>
                  <p><strong>Compatibility:</strong> {aiInsights.moodAnalysis?.compatibility}</p>
                  <p><strong>💡 Suggestion:</strong> {aiInsights.moodAnalysis?.suggestion}</p>
                </div>

                {songs.length > 0 && (
                  <div className="insight-section">
                    <h3>
                      <Music size={20} />
                      Music Compatibility
                    </h3>
                    <p>{aiInsights.musicInsights?.compatibility}</p>
                    <p><strong>🎵 Recommended:</strong> {aiInsights.musicInsights?.recommendation}</p>
                  </div>
                )}

                <div className="insight-section">
                  <h3>
                    <MessageCircle size={20} />
                    Conversation Starters
                  </h3>
                  <ul>
                    {aiInsights.conversationStarters?.map((question, i) => (
                      <li key={i}>{question}</li>
                    ))}
                  </ul>
                </div>

                <div className="insight-section">
                  <h3>
                    <CalendarIcon size={20} />
                    Your Story
                  </h3>
                  <p>{aiInsights.weekSummary}</p>
                </div>
              </>
            ) : null}

            <button className="close-insights" onClick={() => setShowInsights(false)}>
              Close
            </button>
          </div>
        </>
      )}
    </div>
  );
}
