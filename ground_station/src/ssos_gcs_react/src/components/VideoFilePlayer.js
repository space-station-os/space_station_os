import React from 'react';

function VideoFilePlayer() {
  return (
    <div>
      <h2>Cam 1: Video Feed</h2>
      <video 
        src="/output.mp4" 
        controls 
        autoPlay 
        loop 
        muted
        width="420" 
        style={{ border: '1px solid black' }}
      >
        Your browser does not support the video tag.
      </video>
    </div>
  );
}

export default VideoFilePlayer;
