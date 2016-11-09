# Numbered

[![NPM version][npm-image]][npm-url]
[![Build status][travis-image]][travis-url]
[![Test coverage][coveralls-image]][coveralls-url]
[![Gittip][gittip-image]][gittip-url]

Turn any number into a formatted word string, and turn it back again.

## Installation

```
npm install numbered --save
```

## API

```javascript
var numbered = require('numbered');
```

* [numbered( number|string )](#function)
* [numbered.parse( string )](#parse)
* [numbered.stringify( number )](#stringify)

### Function

Number Words exposes a single function that accepts either a string or a number. The string will delegate to the `parse` method and a number will delegate to the `stringify` method.

### Parse

Parses a string into a number as best as possible.

```
numbered.parse('ninety nine');
=> 99

numbered.parse('point two five nine');
=> 0.259
```

### Stringify

Stringifies a number to the word equivalent.

```
numbered.stringify(99);
=> "ninety nine"

numbered.stringify(0.259);
=> "zero point two five nine"
```

## License

MIT

[npm-image]: https://img.shields.io/npm/v/numbered.svg?style=flat
[npm-url]: https://npmjs.org/package/numbered
[travis-image]: https://img.shields.io/travis/blakeembrey/node-numbered.svg?style=flat
[travis-url]: https://travis-ci.org/blakeembrey/node-numbered
[coveralls-image]: https://img.shields.io/coveralls/blakeembrey/node-numbered.svg?style=flat
[coveralls-url]: https://coveralls.io/r/blakeembrey/node-numbered?branch=master
[gittip-image]: https://img.shields.io/gittip/blakeembrey.svg?style=flat
[gittip-url]: https://www.gittip.com/blakeembrey
