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
      days.push(<div key={`empty-${i}`} className="aspect-square" />);
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
          className={`
            aspect-square rounded-lg flex flex-col items-center justify-center
            transition-all duration-200 relative group
            ${isSelected ? 'bg-pink-600 ring-2 ring-pink-400' : ''}
            ${isToday && !isSelected ? 'bg-purple-600 ring-2 ring-purple-400' : ''}
            ${!isToday && !isSelected ? 'hover:bg-purple-900/30' : ''}
            ${hasMemories ? 'font-bold' : ''}
          `}
        >
          <span className="text-sm">{day}</span>
          {hasMemories && (
            <div className="absolute bottom-1 flex gap-0.5">
              {postsOnDay.slice(0, 3).map((_, i) => (
                <div
                  key={i}
                  className="w-1 h-1 rounded-full bg-pink-400"
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
    <div className="glass rounded-2xl p-4 mb-4">
      {/* Header */}
      <div className="flex items-center justify-between mb-4">
        <button
          onClick={prevMonth}
          className="p-2 hover:bg-purple-800/30 rounded-lg transition-colors"
        >
          <ChevronLeft className="w-5 h-5" />
        </button>
        
        <h3 className="font-semibold text-lg">
          {monthNames[currentDate.getMonth()]} {currentDate.getFullYear()}
        </h3>
        
        <button
          onClick={nextMonth}
          className="p-2 hover:bg-purple-800/30 rounded-lg transition-colors"
        >
          <ChevronRight className="w-5 h-5" />
        </button>
      </div>
      
      {/* Day labels */}
      <div className="grid grid-cols-7 gap-1 mb-2">
        {['S', 'M', 'T', 'W', 'T', 'F', 'S'].map((day, i) => (
          <div key={i} className="text-center text-xs text-purple-300 font-medium">
            {day}
          </div>
        ))}
      </div>
      
      {/* Calendar grid */}
      <div className="grid grid-cols-7 gap-1">
        {renderDays()}
      </div>
      
      {/* Legend */}
      <div className="mt-4 flex items-center justify-between text-xs text-purple-300">
        <div className="flex items-center gap-4">
          <div className="flex items-center gap-1">
            <div className="w-2 h-2 rounded-full bg-pink-400" />
            <span>Memory</span>
          </div>
          <div className="flex items-center gap-1">
            <div className="w-2 h-2 rounded-full bg-purple-400 ring-2 ring-purple-400/50" />
            <span>Today</span>
          </div>
        </div>
        {selectedDate && (
          <button
            onClick={onClearFilter}
            className="bg-pink-600 hover:bg-pink-700 px-3 py-1 rounded-full flex items-center gap-1 transition-all"
          >
            <X className="w-3 h-3" />
            <span>Clear Filter</span>
          </button>
        )}
      </div>
    </div>
  );
}
