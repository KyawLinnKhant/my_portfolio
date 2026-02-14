import React from 'react';
import { ChevronLeft, ChevronRight, X } from 'lucide-react';

export default function Calendar({ posts, onDateSelect, selectedDate, onClearFilter }) {
  const [currentDate, setCurrentDate] = React.useState(new Date());
  
  const daysInMonth = new Date(
    currentDate.getFullYear(),
    currentDate.getMonth() + 1,
    0
  ).getDate();
  
  const firstDayOfMonth = new Date(
    currentDate.getFullYear(),
    currentDate.getMonth(),
    1
  ).getDay();
  
  const monthNames = [
    'January', 'February', 'March', 'April', 'May', 'June',
    'July', 'August', 'September', 'October', 'November', 'December'
  ];
  
  const dayNames = ['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat'];
  
  const prevMonth = () => {
    setCurrentDate(new Date(currentDate.getFullYear(), currentDate.getMonth() - 1));
  };
  
  const nextMonth = () => {
    setCurrentDate(new Date(currentDate.getFullYear(), currentDate.getMonth() + 1));
  };
  
  // Get posts by date
  const getPostsForDate = (day) => {
    const dateStr = `${currentDate.getFullYear()}-${String(currentDate.getMonth() + 1).padStart(2, '0')}-${String(day).padStart(2, '0')}`;
    return posts.filter(post => {
      const postDate = new Date(post.timestamp);
      return postDate.toISOString().split('T')[0] === dateStr;
    });
  };
  
  const renderDays = () => {
    const days = [];
    
    // Empty cells for days before month starts
    for (let i = 0; i < firstDayOfMonth; i++) {
      days.push(
        <div key={`empty-${i}`} className="calendar-day-empty" />
      );
    }
    
    // Days of the month
    for (let day = 1; day <= daysInMonth; day++) {
      const postsOnDay = getPostsForDate(day);
      const hasMemories = postsOnDay.length > 0;
      const isToday = 
        day === new Date().getDate() &&
        currentDate.getMonth() === new Date().getMonth() &&
        currentDate.getFullYear() === new Date().getFullYear();
      
      const isSelected = selectedDate &&
        day === selectedDate.getDate() &&
        currentDate.getMonth() === selectedDate.getMonth() &&
        currentDate.getFullYear() === selectedDate.getFullYear();
      
      days.push(
        <button
          key={day}
          onClick={() => {
            const clickedDate = new Date(currentDate.getFullYear(), currentDate.getMonth(), day);
            onDateSelect && onDateSelect(clickedDate);
          }}
          className={`calendar-day ${isSelected ? 'selected' : ''} ${isToday ? 'today' : ''} ${hasMemories ? 'has-memories' : ''}`}
        >
          <span className="day-number">{day}</span>
          {hasMemories && (
            <div className="memory-indicators">
              {postsOnDay.slice(0, 3).map((_, i) => (
                <div
                  key={i}
                  className="memory-dot"
                />
              ))}
            </div>
          )}
        </button>
      );
    }
    
    return days;
  };
  
  return (
    <div className="calendar-container">
      {/* Header */}
      <div className="calendar-header">
        <button
          onClick={prevMonth}
          className="calendar-nav-button"
        >
          <ChevronLeft className="w-5 h-5" />
        </button>
        
        <h3 className="calendar-title">
          {monthNames[currentDate.getMonth()]} {currentDate.getFullYear()}
        </h3>
        
        <button
          onClick={nextMonth}
          className="calendar-nav-button"
        >
          <ChevronRight className="w-5 h-5" />
        </button>
      </div>
      
      {/* Day labels */}
      <div className="calendar-grid">
        {dayNames.map((day, i) => (
          <div key={i} className="calendar-day-label">
            {day}
          </div>
        ))}
        
        {/* Calendar days */}
        {renderDays()}
      </div>
      
      {/* Legend */}
      <div className="calendar-legend">
        <div className="legend-items">
          <div className="legend-item">
            <div className="legend-icon memory" />
            <span>Memory</span>
          </div>
          <div className="legend-item">
            <div className="legend-icon today" />
            <span>Today</span>
          </div>
        </div>
        {selectedDate && (
          <button
            onClick={onClearFilter}
            className="clear-filter-button"
          >
            <X className="w-3 h-3" />
            <span>Clear</span>
          </button>
        )}
      </div>

      <style jsx>{`
        .calendar-container {
          background: rgba(139, 92, 246, 0.1);
          backdrop-filter: blur(10px);
          border: 1px solid rgba(139, 92, 246, 0.2);
          border-radius: 1rem;
          padding: 1rem;
        }

        .calendar-header {
          display: flex;
          align-items: center;
          justify-content: space-between;
          margin-bottom: 1rem;
        }

        .calendar-nav-button {
          background: rgba(168, 85, 247, 0.2);
          border: none;
          padding: 0.5rem;
          border-radius: 0.5rem;
          cursor: pointer;
          color: white;
          transition: all 0.2s;
          display: flex;
          align-items: center;
          justify-content: center;
        }

        .calendar-nav-button:hover {
          background: rgba(168, 85, 247, 0.4);
        }

        .calendar-title {
          font-size: 1.125rem;
          font-weight: 600;
          color: white;
        }

        .calendar-grid {
          display: grid;
          grid-template-columns: repeat(7, 1fr);
          gap: 0.25rem;
        }

        .calendar-day-label {
          text-align: center;
          font-size: 0.75rem;
          color: rgba(168, 85, 247, 0.8);
          font-weight: 600;
          padding: 0.5rem 0;
        }

        .calendar-day-empty {
          aspect-ratio: 1;
        }

        .calendar-day {
          aspect-ratio: 1;
          background: none;
          border: none;
          border-radius: 0.5rem;
          cursor: pointer;
          color: rgba(255, 255, 255, 0.7);
          font-size: 0.875rem;
          display: flex;
          flex-direction: column;
          align-items: center;
          justify-content: center;
          position: relative;
          transition: all 0.2s;
          padding: 0.25rem;
        }

        .calendar-day:hover {
          background: rgba(168, 85, 247, 0.2);
          color: white;
        }

        .calendar-day.has-memories {
          color: white;
          font-weight: 600;
        }

        .calendar-day.today {
          background: rgba(168, 85, 247, 0.4);
          border: 2px solid rgba(168, 85, 247, 0.6);
          color: white;
        }

        .calendar-day.selected {
          background: linear-gradient(135deg, #ec4899 0%, #8b5cf6 100%);
          border: 2px solid #ec4899;
          color: white;
        }

        .day-number {
          position: relative;
          z-index: 1;
        }

        .memory-indicators {
          position: absolute;
          bottom: 4px;
          display: flex;
          gap: 2px;
        }

        .memory-dot {
          width: 4px;
          height: 4px;
          border-radius: 50%;
          background: #ec4899;
        }

        .calendar-legend {
          margin-top: 1rem;
          display: flex;
          align-items: center;
          justify-content: space-between;
          font-size: 0.75rem;
          color: rgba(255, 255, 255, 0.7);
        }

        .legend-items {
          display: flex;
          gap: 1rem;
        }

        .legend-item {
          display: flex;
          align-items: center;
          gap: 0.5rem;
        }

        .legend-icon {
          width: 12px;
          height: 12px;
          border-radius: 50%;
        }

        .legend-icon.memory {
          background: #ec4899;
        }

        .legend-icon.today {
          background: rgba(168, 85, 247, 0.6);
          border: 2px solid rgba(168, 85, 247, 0.8);
        }

        .clear-filter-button {
          background: linear-gradient(135deg, #ec4899 0%, #8b5cf6 100%);
          border: none;
          padding: 0.375rem 0.75rem;
          border-radius: 9999px;
          color: white;
          font-size: 0.75rem;
          font-weight: 600;
          cursor: pointer;
          display: flex;
          align-items: center;
          gap: 0.25rem;
          transition: all 0.2s;
        }

        .clear-filter-button:hover {
          transform: scale(1.05);
          box-shadow: 0 4px 12px rgba(236, 72, 153, 0.4);
        }
      `}</style>
    </div>
  );
}
