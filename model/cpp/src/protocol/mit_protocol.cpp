#include "protocol/mit_protocol.hpp"

namespace protocol {

// 由於所有核心壓碼/解碼邏輯皆為 constexpr / header-inline 實作，
// 此處作為編譯單元 (Translation Unit) 錨點，方便日後擴充非內聯通訊驗證與統計邏輯。

} // namespace protocol