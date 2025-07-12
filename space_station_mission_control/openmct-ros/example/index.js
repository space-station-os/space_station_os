
import installRosPlugin from '../src/plugin.js';

const config = {
    "rosBridgeEndpoint": "ws://localhost:9090",
    "unsubscribeFromTopicsOnStop": false,
    "telemetryDataToKeepPerTopic": 10000,
    "flattenArraysToSize": 10,
    "topicDetailsBatchSize": 5,
    "namespace": "ros"
};
const openmct = window.openmct;

(function () {
    const THIRTY_MINUTES = 30 * 60 * 1000;

    openmct.setAssetPath('/node_modules/openmct/dist');

    installDefaultPlugins();
    openmct.install(installRosPlugin(config));

    openmct.start();

    function installDefaultPlugins() {
        openmct.install(openmct.plugins.LocalStorage());
        openmct.install(openmct.plugins.Espresso());
        openmct.install(openmct.plugins.MyItems());
        openmct.install(openmct.plugins.example.Generator());
        openmct.install(openmct.plugins.example.ExampleImagery());
        openmct.install(openmct.plugins.UTCTimeSystem());
        openmct.install(openmct.plugins.TelemetryMean());

        openmct.install(openmct.plugins.DisplayLayout({
            showAsView: ['summary-widget', 'example.imagery']
        }));
        openmct.install(openmct.plugins.Conductor({
            menuOptions: [
                {
                    name: "Realtime",
                    timeSystem: 'utc',
                    clock: 'local',
                    clockOffsets: {
                        start: -THIRTY_MINUTES,
                        end: 0
                    }
                },
                {
                    name: "Fixed",
                    timeSystem: 'utc',
                    bounds: {
                        start: Date.now() - THIRTY_MINUTES,
                        end: 0
                    }
                }
            ]
        }));
        openmct.install(openmct.plugins.SummaryWidget());
        openmct.install(openmct.plugins.Notebook());
        openmct.install(openmct.plugins.LADTable());
        openmct.install(openmct.plugins.ClearData(['table', 'telemetry.plot.overlay', 'telemetry.plot.stacked']));
    }
}());
