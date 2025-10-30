/*!
 * vue-router v4.6.3
 * (c) 2025 Eduardo San Martin Morote
 * @license MIT
 */
import { EXPERIMENTAL_RouteRecordNormalized, EXPERIMENTAL_RouteRecordNormalized_Group, EXPERIMENTAL_RouteRecordNormalized_Matchable, EXPERIMENTAL_RouteRecordRaw, EXPERIMENTAL_RouteRecord_Base, EXPERIMENTAL_RouteRecord_Group, EXPERIMENTAL_RouteRecord_Matchable, EXPERIMENTAL_Router, EXPERIMENTAL_RouterOptions, EXPERIMENTAL_RouterOptions_Base, EXPERIMENTAL_Router_Base, EmptyParams, MatcherParamsFormatted, MatcherPattern, MatcherPatternHash, MatcherPatternPath, MatcherPatternPathDynamic, MatcherPatternPathDynamic_ParamOptions, MatcherPatternPathStatic, MatcherPatternQuery, MatcherPatternQueryParam, MatcherQueryParams, MatcherQueryParamsValue, PARAM_PARSER_BOOL, PARAM_PARSER_INT, ParamParser, createFixedResolver, defineParamParser, definePathParamParser, defineQueryParamParser, experimental_createRouter, mergeRouteRecord, normalizeRouteRecord } from "../router-BbqN7H95.mjs";

//#region src/experimental/route-resolver/matchers/errors.d.ts
/**
 * Error throw when a matcher matches by regex but validation fails.
 */
declare class MatchMiss extends Error {
  name: string;
}
/**
 * Helper to create a {@link MatchMiss} error.
 * @param args - Arguments to pass to the `MatchMiss` constructor.
 *
 * @example
 * ```ts
 * throw miss()
 * // in a number param matcher
 * throw miss('Number must be finite')
 * ```
 */
declare const miss: (...args: ConstructorParameters<typeof MatchMiss>) => MatchMiss;
//#endregion
//#region src/experimental/index.d.ts
declare module 'vue-router' {
  interface RouteLocationMatched {
    /**
     * The experimental router uses a `parent` property instead of `children`.
     */
    children?: never;
  }
}
//#endregion
export { type EXPERIMENTAL_RouteRecordNormalized, type EXPERIMENTAL_RouteRecordNormalized_Group, type EXPERIMENTAL_RouteRecordNormalized_Matchable, type EXPERIMENTAL_RouteRecordRaw, type EXPERIMENTAL_RouteRecord_Base, type EXPERIMENTAL_RouteRecord_Group, type EXPERIMENTAL_RouteRecord_Matchable, type EXPERIMENTAL_Router, type EXPERIMENTAL_RouterOptions, type EXPERIMENTAL_RouterOptions_Base, type EXPERIMENTAL_Router_Base, type EmptyParams, MatchMiss, type MatcherParamsFormatted, type MatcherPattern, type MatcherPatternHash, type MatcherPatternPath, MatcherPatternPathDynamic, type MatcherPatternPathDynamic_ParamOptions, MatcherPatternPathStatic, type MatcherPatternQuery, MatcherPatternQueryParam, type MatcherQueryParams, type MatcherQueryParamsValue, PARAM_PARSER_BOOL, PARAM_PARSER_INT, type ParamParser, mergeRouteRecord as _mergeRouteRecord, createFixedResolver, defineParamParser, definePathParamParser, defineQueryParamParser, experimental_createRouter, miss, normalizeRouteRecord };