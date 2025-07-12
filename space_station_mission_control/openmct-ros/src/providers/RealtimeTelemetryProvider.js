import { OBJECT_TYPES } from '../const';
import ROSLIB from 'roslib';
import { LRUCache } from 'lru-cache';

export default class RealtimeTelemetryProvider {
    constructor(openmct, rosConnection, unsubscribeFromTopicsOnStop, telemetryDataToKeepPerTopic) {
        this.openmct = openmct;
        this.rosConnection = rosConnection;
        this.subscriptionsById = {};
        this.telemetryDataToKeepPerTopic = telemetryDataToKeepPerTopic;
        this.unsubscribeFromTopicsOnStop = unsubscribeFromTopicsOnStop;
    }
    // eslint-disable-next-line require-await
    async request(domainObject, options) {
        const startTelemetry = options.start;
        const endTelemetry = options.end;
        const subscriberID = domainObject.identifier.key;
        const cache = this.subscriptionsById[subscriberID]?.cache;
        // from the cache, extract all the values that are within the requested range
        let dataToBeReturned = [];
        const cacheIterator = cache ? cache.values() : null;

        if (cacheIterator) {
            let next = cacheIterator.next();
            while (!next.done) {
                if ((next.value) && (next.value.timestamp >= startTelemetry && next.value.timestamp <= endTelemetry)) {
                    const datum = {
                        id: domainObject.identifier,
                        value: next.value.value,
                        timestamp: next.value.timestamp
                    };
                    dataToBeReturned.push(datum);
                }

                next = cacheIterator.next();
            }
        }

        return dataToBeReturned;
    }
    async #buildSubscription(domainObject, callback) {
        if (this.subscriptionsById[domainObject.identifier.key]) {
            return this.subscriptionsById[domainObject.identifier.key];
        }

        const id = domainObject.identifier.key;
        const rosTopicName = domainObject.rosTopic.replace(/\./g, '/');
        const ros = await this.rosConnection.getConnection();
        const topic = new ROSLIB.Topic({
            ros,
            name: `/${rosTopicName}`,
            messageType: domainObject.rosType
        });

        return {
            topic,
            id,
            topicName: rosTopicName,
            callback,
            cache: new LRUCache({max: this.telemetryDataToKeepPerTopic})
        };
    }

    supportsSubscribe(domainObject) {
        return Object.values(OBJECT_TYPES).includes(domainObject.type);
    }

    supportsRequest(domainObject) {
        return Object.values(OBJECT_TYPES).includes(domainObject.type);
    }

    subscribe(domainObject, callback) {
        const subscriberID = domainObject.identifier.key;

        this.#buildSubscription(domainObject, callback).then(subscriptionDetails => {
            subscriptionDetails.topic.subscribe((data) => {
                // value is the addressed value in the message
                // e.g., for a message like {linear: { x: 5, y: 2, z: 4 } }
                // and an id of someTopic.linear._x, the value is just 5

                const splitId = subscriberID.split('.');

                // parse as deep as necessary, not just the last key
                let value = data;
                splitId.forEach(key => {
                    key = key.replace('_', '');
                    if (Array.isArray(value) && key >= value.length) {
                        value = undefined;
                    } else if (value && value[key] !== undefined) {
                        value = value[key];
                    }
                });

                if (value !== undefined && (typeof value !== 'object')) {
                    const timestamp = Date.now();
                    const datum = {
                        id: domainObject.identifier,
                        timestamp,
                        value: value.toString()
                    };
                    subscriptionDetails.cache.set(timestamp, datum);
                    callback(datum);
                }
            });

            this.subscriptionsById[subscriberID] = subscriptionDetails;
        });

        return () => {
            if (this.unsubscribeFromTopicsOnStop) {
                this.subscriptionsById[subscriberID].topic.unsubscribe();
                delete this.subscriptionsById[subscriberID];
            }
        };
    }
}
