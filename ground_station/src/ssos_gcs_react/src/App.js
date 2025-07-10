import React from "react";
import YamcsDataPanel from "./components/YamcsDataPanel";
import CommandForm from "./components/CommandForm";
import VideoFilePlayer from "./components/VideoFilePlayer";
import LivePlot from "./components/LivePlot";
import MongoDataViewer from "./components/MongoDataViewer";

import "./App.css";

function App() {
  return (
    <div className="app-container">
      <header className="global-header">
        Space Station OS - Virtual Ground Control Station (SSOS-vGCS)
      </header>
      <div className="container">
        {/* LEFT COLUMN: split into top, middle, and bottom */}
        <div className="left-panel">
          <div className="left-top">
            <YamcsDataPanel />
          </div>
          <div className="left-middle">
            <MongoDataViewer />
          </div>
          <div className="left-bottom">
            <CommandForm />
          </div>
        </div>

        {/* RIGHT COLUMN: split into top and bottom */}
        <div className="right-panel">
          <div className="right-top">
            <VideoFilePlayer />
          </div>
          <div className="right-middle">
            <LivePlot />
          </div>
        </div>
      </div>
    </div>
  );
}

export default App;
