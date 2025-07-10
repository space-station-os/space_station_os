import { OBJECT_TYPES, ROOT_KEY } from '../const';

// eslint-disable-next-line no-unused-vars
const formatConversionMap = {
    "uint64": "int",
    "int64": "int",
    "uint32": "int",
    "int32": "int",
    "uint16": "int",
    "int16": "int",
    "uint8": "int",
    "int8": "int",

    "float32": "float",
    "float64": "float",
    "float": "float",
    "double": "float",

    "byte": "byte",
    "string": "string",
    "bool": "bool"
};

export default class RosObjectProvider {
    constructor(openmct, rosConnection, namespace, flattenArraysToSize, topicDetailsBatchSize) {
        this.openmct = openmct;
        this.rosConnection = rosConnection;
        this.namespace = namespace;
        this.rootObject = null;
        this.dictionary = {};
        this.fetchRosTopicsPromise = null;
        this.flattenArraysToSize = flattenArraysToSize;
        this.topicDetailsBatchSize = topicDetailsBatchSize;
        this.messageDetailsCache = {};

        this.#initialize();
    }

    #initialize() {
        this.rootObject = this.#createRootObject();
    }

    async getRosTopics(ros) {
        const result = await new Promise((resolve, reject) => {
            ros.getTopics((topics) => {
                resolve(topics);
            }, (error) => {
                reject(error);
            });
        });

        return result;
    }

    async #fetchFromRos() {
        const ros = await this.rosConnection.getConnection();
        const { topics, types } = await this.getRosTopics(ros);

        for (let i = 0; i < topics.length; i += this.topicDetailsBatchSize) {
            const batchTopics = topics.slice(i, i + this.topicDetailsBatchSize);
            const batchTypes = types.slice(i, i + this.topicDetailsBatchSize);

            // wait for this batch to complete before firing the next
            await Promise.all(batchTopics.map(async (topic, idx) => {
                const messageType = batchTypes[idx];
                const messageDetails = await this.#getMessageDetails(ros, messageType);
                const name = topic.replace(/\//g, '.').slice(1);

                this.#addRosTelemetry({
                    name,
                    type: OBJECT_TYPES.ROS_TOPIC_TYPE,
                    messageDetails,
                    rosType: messageType,
                    rosTopic: name,
                    parent: this.rootObject
                });
            }));
        }
    }

    #getMessageDetails(ros, messageType) {
        // Return cached promise if it exists
        if (this.messageDetailsCache[messageType]) {
            return this.messageDetailsCache[messageType];
        }

        // Otherwise, create, cache, and return a new promise
        this.messageDetailsCache[messageType] = new Promise((resolve, reject) => {
            ros.getMessageDetails(
                messageType,
                (details) => {
                    const decodeMessageDetails = ros.decodeTypeDefs(details);
                    resolve(decodeMessageDetails);
                },
                (error) => {
                    console.error(`🚨 Error fetching message details for ${messageType}`, error);
                    reject(error);
                }
            );
        });

        return this.messageDetailsCache[messageType];
    }

    #loadRosDictionary() {
        if (!this.fetchRosTopicsPromise) {
            this.fetchRosTopicsPromise = this.#fetchFromRos();
        }

        return this.fetchRosTopicsPromise;
    }

    #createRootObject() {
        const rootObject = {
            identifier: {
                key: ROOT_KEY,
                namespace: this.namespace
            },
            name: 'ROS Topics',
            type: 'folder',
            location: 'ROOT',
            composition: []
        };
        this.#addObject(rootObject);

        return rootObject;
    }

    async get(identifier) {
        const { key } = identifier;
        await this.#loadRosDictionary();
        const object = this.dictionary[key];

        return object;
    }

    supportsSearchType(type) {
        // return type === this.openmct.objects.SEARCH_TYPES.OBJECTS;
        // TODO: support object search by querying ROS
        return false;
    }

    // eslint-disable-next-line require-await
    async search(query, options) {
        return [];
    }

    #addObject(object) {
        this.dictionary[object.identifier.key] = object;
    }

    #isAggregateMessage(type) {
        return type === OBJECT_TYPES.ROS_AGGREGATE_MESSAGE || type === OBJECT_TYPES.ROS_TOPIC_TYPE;
    }

    #addRosTelemetry({name, type, rosType, rosTopic, messageDetails, parent}) {
        const location = this.openmct.objects.makeKeyString({
            key: parent.identifier.key,
            namespace: parent.identifier.namespace
        });
        const subMessageKeys = typeof messageDetails === 'object' ? Object.keys(messageDetails) : [];

        let determinedType = type;
        // if type is null, determine if we're a lead or aggregate message
        if (!determinedType) {
            if (subMessageKeys.length > 0) {
                determinedType = OBJECT_TYPES.ROS_AGGREGATE_MESSAGE;
            } else {
                determinedType = OBJECT_TYPES.ROS_LEAF_MESSAGE;
            }
        }

        let determinedName = name;
        if (determinedType !== OBJECT_TYPES.ROS_TOPIC_TYPE) {
            determinedName = `${parent.identifier.key}.${name}`;
        }

        const newMctObject = {
            identifier: {
                key: determinedName,
                namespace: this.namespace
            },
            type: determinedType,
            name: determinedName,
            rosType,
            rosTopic,
            location: location,
            configuration: {},
            telemetry: {
                values: [{
                    key: 'utc',
                    source: 'timestamp',
                    name: 'Timestamp',
                    format: 'iso',
                    hints: {
                        domain: 1
                    }
                }]
            }
        };

        if (this.#isAggregateMessage(determinedType)) {
            subMessageKeys.forEach((subMessageKey) => {
                let subMessageDetail = messageDetails[subMessageKey];
                if (Array.isArray(subMessageDetail)) {
                    // replace subMessageDetail with X elements equal to this.flattenArraysToSize
                    subMessageDetail = new Array(this.flattenArraysToSize).fill(subMessageDetail[0]);
                }

                const format = formatConversionMap[subMessageDetail];
                if (format) {
                    const telemetryValue = {
                        key: `${determinedName}.${subMessageKey}`,
                        source: subMessageKey,
                        name: subMessageKey,
                        format,
                        rosType: subMessageDetail,
                        hints: {
                            domain: 1
                        }
                    };
                    newMctObject.telemetry.values.push(telemetryValue);
                }

                this.#addRosTelemetry({
                    name: subMessageKey,
                    messageDetails: subMessageDetail,
                    rosType,
                    rosTopic,
                    parent: newMctObject
                });
            });
        } else {
            // leaf node
            const format = formatConversionMap[messageDetails];
            if (format) {
                const telemetryValue = {
                    key: 'value',
                    name: 'Value',
                    format,
                    rosType: messageDetails,
                    hints: {
                        range: 1
                    }
                };
                newMctObject.telemetry.values.push(telemetryValue);
            }
        }

        this.#addObject(newMctObject);

        if (!parent.composition) {
            parent.composition = [];
        }

        parent.composition.push(newMctObject.identifier);
    }
}
