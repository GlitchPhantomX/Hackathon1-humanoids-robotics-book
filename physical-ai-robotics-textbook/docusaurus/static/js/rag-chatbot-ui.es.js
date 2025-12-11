import be, { forwardRef as de, createElement as K, memo as he, useState as j, useEffect as N, useRef as ae, useCallback as H } from "react";
var Y = { exports: {} }, M = {};
var re;
function ge() {
  if (re) return M;
  re = 1;
  var t = Symbol.for("react.transitional.element"), a = Symbol.for("react.fragment");
  function n(o, c, i) {
    var l = null;
    if (i !== void 0 && (l = "" + i), c.key !== void 0 && (l = "" + c.key), "key" in c) {
      i = {};
      for (var u in c)
        u !== "key" && (i[u] = c[u]);
    } else i = c;
    return c = i.ref, {
      $$typeof: t,
      type: o,
      key: l,
      ref: c !== void 0 ? c : null,
      props: i
    };
  }
  return M.Fragment = a, M.jsx = n, M.jsxs = n, M;
}
var O = {};
var ne;
function xe() {
  return ne || (ne = 1, process.env.NODE_ENV !== "production" && (function() {
    function t(e) {
      if (e == null) return null;
      if (typeof e == "function")
        return e.$$typeof === R ? null : e.displayName || e.name || null;
      if (typeof e == "string") return e;
      switch (e) {
        case k:
          return "Fragment";
        case w:
          return "Profiler";
        case I:
          return "StrictMode";
        case L:
          return "Suspense";
        case b:
          return "SuspenseList";
        case W:
          return "Activity";
      }
      if (typeof e == "object")
        switch (typeof e.tag == "number" && console.error(
          "Received an unexpected object in getComponentNameFromType(). This is likely a bug in React. Please file an issue."
        ), e.$$typeof) {
          case P:
            return "Portal";
          case V:
            return e.displayName || "Context";
          case $:
            return (e._context.displayName || "Context") + ".Consumer";
          case F:
            var r = e.render;
            return e = e.displayName, e || (e = r.displayName || r.name || "", e = e !== "" ? "ForwardRef(" + e + ")" : "ForwardRef"), e;
          case x:
            return r = e.displayName || null, r !== null ? r : t(e.type) || "Memo";
          case S:
            r = e._payload, e = e._init;
            try {
              return t(e(r));
            } catch {
            }
        }
      return null;
    }
    function a(e) {
      return "" + e;
    }
    function n(e) {
      try {
        a(e);
        var r = !1;
      } catch {
        r = !0;
      }
      if (r) {
        r = console;
        var d = r.error, h = typeof Symbol == "function" && Symbol.toStringTag && e[Symbol.toStringTag] || e.constructor.name || "Object";
        return d.call(
          r,
          "The provided key is an unsupported type %s. This value must be coerced to a string before using it here.",
          h
        ), a(e);
      }
    }
    function o(e) {
      if (e === k) return "<>";
      if (typeof e == "object" && e !== null && e.$$typeof === S)
        return "<...>";
      try {
        var r = t(e);
        return r ? "<" + r + ">" : "<...>";
      } catch {
        return "<...>";
      }
    }
    function c() {
      var e = E.A;
      return e === null ? null : e.getOwner();
    }
    function i() {
      return Error("react-stack-top-frame");
    }
    function l(e) {
      if (D.call(e, "key")) {
        var r = Object.getOwnPropertyDescriptor(e, "key").get;
        if (r && r.isReactWarning) return !1;
      }
      return e.key !== void 0;
    }
    function u(e, r) {
      function d() {
        Z || (Z = !0, console.error(
          "%s: `key` is not a prop. Trying to access it will result in `undefined` being returned. If you need to access the same value within the child component, you should pass it as a different prop. (https://react.dev/link/special-props)",
          r
        ));
      }
      d.isReactWarning = !0, Object.defineProperty(e, "key", {
        get: d,
        configurable: !0
      });
    }
    function m() {
      var e = t(this.type);
      return Q[e] || (Q[e] = !0, console.error(
        "Accessing element.ref was removed in React 19. ref is now a regular prop. It will be removed from the JSX Element type in a future release."
      )), e = this.props.ref, e !== void 0 ? e : null;
    }
    function g(e, r, d, h, z, G) {
      var f = d.ref;
      return e = {
        $$typeof: C,
        type: e,
        key: r,
        props: d,
        _owner: h
      }, (f !== void 0 ? f : null) !== null ? Object.defineProperty(e, "ref", {
        enumerable: !1,
        get: m
      }) : Object.defineProperty(e, "ref", { enumerable: !1, value: null }), e._store = {}, Object.defineProperty(e._store, "validated", {
        configurable: !1,
        enumerable: !1,
        writable: !0,
        value: 0
      }), Object.defineProperty(e, "_debugInfo", {
        configurable: !1,
        enumerable: !1,
        writable: !0,
        value: null
      }), Object.defineProperty(e, "_debugStack", {
        configurable: !1,
        enumerable: !1,
        writable: !0,
        value: z
      }), Object.defineProperty(e, "_debugTask", {
        configurable: !1,
        enumerable: !1,
        writable: !0,
        value: G
      }), Object.freeze && (Object.freeze(e.props), Object.freeze(e)), e;
    }
    function p(e, r, d, h, z, G) {
      var f = r.children;
      if (f !== void 0)
        if (h)
          if (q(f)) {
            for (h = 0; h < f.length; h++)
              y(f[h]);
            Object.freeze && Object.freeze(f);
          } else
            console.error(
              "React.jsx: Static children should always be an array. You are likely explicitly calling React.jsxs or React.jsxDEV. Use the Babel transform instead."
            );
        else y(f);
      if (D.call(r, "key")) {
        f = t(e);
        var A = Object.keys(r).filter(function(pe) {
          return pe !== "key";
        });
        h = 0 < A.length ? "{key: someKey, " + A.join(": ..., ") + ": ...}" : "{key: someKey}", se[f + h] || (A = 0 < A.length ? "{" + A.join(": ..., ") + ": ...}" : "{}", console.error(
          `A props object containing a "key" prop is being spread into JSX:
  let props = %s;
  <%s {...props} />
React keys must be passed directly to JSX without using spread:
  let props = %s;
  <%s key={someKey} {...props} />`,
          h,
          f,
          A,
          f
        ), se[f + h] = !0);
      }
      if (f = null, d !== void 0 && (n(d), f = "" + d), l(r) && (n(r.key), f = "" + r.key), "key" in r) {
        d = {};
        for (var B in r)
          B !== "key" && (d[B] = r[B]);
      } else d = r;
      return f && u(
        d,
        typeof e == "function" ? e.displayName || e.name || "Unknown" : e
      ), g(
        e,
        f,
        d,
        c(),
        z,
        G
      );
    }
    function y(e) {
      T(e) ? e._store && (e._store.validated = 1) : typeof e == "object" && e !== null && e.$$typeof === S && (e._payload.status === "fulfilled" ? T(e._payload.value) && e._payload.value._store && (e._payload.value._store.validated = 1) : e._store && (e._store.validated = 1));
    }
    function T(e) {
      return typeof e == "object" && e !== null && e.$$typeof === C;
    }
    var _ = be, C = Symbol.for("react.transitional.element"), P = Symbol.for("react.portal"), k = Symbol.for("react.fragment"), I = Symbol.for("react.strict_mode"), w = Symbol.for("react.profiler"), $ = Symbol.for("react.consumer"), V = Symbol.for("react.context"), F = Symbol.for("react.forward_ref"), L = Symbol.for("react.suspense"), b = Symbol.for("react.suspense_list"), x = Symbol.for("react.memo"), S = Symbol.for("react.lazy"), W = Symbol.for("react.activity"), R = Symbol.for("react.client.reference"), E = _.__CLIENT_INTERNALS_DO_NOT_USE_OR_WARN_USERS_THEY_CANNOT_UPGRADE, D = Object.prototype.hasOwnProperty, q = Array.isArray, U = console.createTask ? console.createTask : function() {
      return null;
    };
    _ = {
      react_stack_bottom_frame: function(e) {
        return e();
      }
    };
    var Z, Q = {}, ee = _.react_stack_bottom_frame.bind(
      _,
      i
    )(), te = U(o(i)), se = {};
    O.Fragment = k, O.jsx = function(e, r, d) {
      var h = 1e4 > E.recentlyCreatedOwnerStacks++;
      return p(
        e,
        r,
        d,
        !1,
        h ? Error("react-stack-top-frame") : ee,
        h ? U(o(e)) : te
      );
    }, O.jsxs = function(e, r, d) {
      var h = 1e4 > E.recentlyCreatedOwnerStacks++;
      return p(
        e,
        r,
        d,
        !0,
        h ? Error("react-stack-top-frame") : ee,
        h ? U(o(e)) : te
      );
    };
  })()), O;
}
var oe;
function ve() {
  return oe || (oe = 1, process.env.NODE_ENV === "production" ? Y.exports = ge() : Y.exports = xe()), Y.exports;
}
var s = ve();
const ye = (t) => t.replace(/([a-z0-9])([A-Z])/g, "$1-$2").toLowerCase(), _e = (t) => t.replace(
  /^([A-Z])|[\s-_]+(\w)/g,
  (a, n, o) => o ? o.toUpperCase() : n.toLowerCase()
), ce = (t) => {
  const a = _e(t);
  return a.charAt(0).toUpperCase() + a.slice(1);
}, fe = (...t) => t.filter((a, n, o) => !!a && a.trim() !== "" && o.indexOf(a) === n).join(" ").trim(), ke = (t) => {
  for (const a in t)
    if (a.startsWith("aria-") || a === "role" || a === "title")
      return !0;
};
var we = {
  xmlns: "http://www.w3.org/2000/svg",
  width: 24,
  height: 24,
  viewBox: "0 0 24 24",
  fill: "none",
  stroke: "currentColor",
  strokeWidth: 2,
  strokeLinecap: "round",
  strokeLinejoin: "round"
};
const Ee = de(
  ({
    color: t = "currentColor",
    size: a = 24,
    strokeWidth: n = 2,
    absoluteStrokeWidth: o,
    className: c = "",
    children: i,
    iconNode: l,
    ...u
  }, m) => K(
    "svg",
    {
      ref: m,
      ...we,
      width: a,
      height: a,
      stroke: t,
      strokeWidth: o ? Number(n) * 24 / Number(a) : n,
      className: fe("lucide", c),
      ...!i && !ke(u) && { "aria-hidden": "true" },
      ...u
    },
    [
      ...l.map(([g, p]) => K(g, p)),
      ...Array.isArray(i) ? i : [i]
    ]
  )
);
const v = (t, a) => {
  const n = de(
    ({ className: o, ...c }, i) => K(Ee, {
      ref: i,
      iconNode: a,
      className: fe(
        `lucide-${ye(ce(t))}`,
        `lucide-${t}`,
        o
      ),
      ...c
    })
  );
  return n.displayName = ce(t), n;
};
const je = [
  ["path", { d: "M12 7v14", key: "1akyts" }],
  [
    "path",
    {
      d: "M3 18a1 1 0 0 1-1-1V4a1 1 0 0 1 1-1h5a4 4 0 0 1 4 4 4 4 0 0 1 4-4h5a1 1 0 0 1 1 1v13a1 1 0 0 1-1 1h-6a3 3 0 0 0-3 3 3 3 0 0 0-3-3z",
      key: "ruj8y"
    }
  ]
], Se = v("book-open", je);
const Re = [
  ["path", { d: "M12 8V4H8", key: "hb8ula" }],
  ["rect", { width: "16", height: "12", x: "4", y: "8", rx: "2", key: "enze0r" }],
  ["path", { d: "M2 14h2", key: "vft8re" }],
  ["path", { d: "M20 14h2", key: "4cs60a" }],
  ["path", { d: "M15 13v2", key: "1xurst" }],
  ["path", { d: "M9 13v2", key: "rq6x2g" }]
], Te = v("bot", Re);
const Ae = [
  ["circle", { cx: "12", cy: "12", r: "10", key: "1mglay" }],
  ["line", { x1: "12", x2: "12", y1: "8", y2: "12", key: "1pkeuh" }],
  ["line", { x1: "12", x2: "12.01", y1: "16", y2: "16", key: "4dfq90" }]
], Ne = v("circle-alert", Ae);
const Ce = [
  ["path", { d: "M15 3h6v6", key: "1q9fwt" }],
  ["path", { d: "M10 14 21 3", key: "gplh6r" }],
  ["path", { d: "M18 13v6a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h6", key: "a6xqqp" }]
], Me = v("external-link", Ce);
const Oe = [
  [
    "path",
    {
      d: "M2.992 16.342a2 2 0 0 1 .094 1.167l-1.065 3.29a1 1 0 0 0 1.236 1.168l3.413-.998a2 2 0 0 1 1.099.092 10 10 0 1 0-4.777-4.719",
      key: "1sd12s"
    }
  ]
], Pe = v("message-circle", Oe);
const Ie = [
  ["circle", { cx: "6", cy: "6", r: "3", key: "1lh9wr" }],
  ["path", { d: "M8.12 8.12 12 12", key: "1alkpv" }],
  ["path", { d: "M20 4 8.12 15.88", key: "xgtan2" }],
  ["circle", { cx: "6", cy: "18", r: "3", key: "fqmcym" }],
  ["path", { d: "M14.8 14.8 20 20", key: "ptml3r" }]
], $e = v("scissors", Ie);
const De = [
  [
    "path",
    {
      d: "M14.536 21.686a.5.5 0 0 0 .937-.024l6.5-19a.496.496 0 0 0-.635-.635l-19 6.5a.5.5 0 0 0-.024.937l7.93 3.18a2 2 0 0 1 1.112 1.11z",
      key: "1ffxy3"
    }
  ],
  ["path", { d: "m21.854 2.147-10.94 10.939", key: "12cjpa" }]
], ze = v("send", De);
const Ye = [
  ["path", { d: "M10 11v6", key: "nco0om" }],
  ["path", { d: "M14 11v6", key: "outv1u" }],
  ["path", { d: "M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6", key: "miytrc" }],
  ["path", { d: "M3 6h18", key: "d0wm0j" }],
  ["path", { d: "M8 6V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2", key: "e791ji" }]
], Ve = v("trash-2", Ye);
const Fe = [
  ["path", { d: "M18 6 6 18", key: "1bl5f8" }],
  ["path", { d: "m6 6 12 12", key: "d8bk6v" }]
], ie = v("x", Fe);
function Le({
  isOpen: t,
  showPulse: a,
  onClick: n
}) {
  return /* @__PURE__ */ s.jsx(
    "button",
    {
      className: `chatbot-toggle-button ${a ? "chatbot-toggle-button--pulse" : ""} ${t ? "chatbot-toggle-button--open" : ""}`,
      onClick: n,
      "aria-label": t ? "Close chat assistant" : "Open chat assistant",
      "aria-expanded": t,
      children: /* @__PURE__ */ s.jsx(Pe, { size: 28 })
    }
  );
}
const We = he(Le);
function qe({ message: t }) {
  const { role: a, content: n, sources: o } = t;
  return /* @__PURE__ */ s.jsxs("div", { className: `chatbot-message chatbot-message--${a}`, children: [
    /* @__PURE__ */ s.jsx("div", { className: "chatbot-message__content", children: n }),
    o && o.length > 0 && /* @__PURE__ */ s.jsx("div", { className: "chatbot-message__sources", children: o.map((c, i) => /* @__PURE__ */ s.jsxs(
      "a",
      {
        href: c.url,
        className: "chatbot-message__source",
        target: "_blank",
        rel: "noopener noreferrer",
        children: [
          /* @__PURE__ */ s.jsx(Me, { size: 14 }),
          /* @__PURE__ */ s.jsx("span", { children: c.title })
        ]
      },
      i
    )) })
  ] });
}
const Ue = he(qe), me = "rag-chatbot-conversations", Ge = 30;
function Be() {
  return "default-conversation";
}
function He() {
  const t = /* @__PURE__ */ new Date();
  return t.setDate(t.getDate() + Ge), t.toISOString();
}
function J() {
  try {
    const t = localStorage.getItem(me);
    if (!t)
      return { conversations: {} };
    const a = JSON.parse(t), n = /* @__PURE__ */ new Date(), o = {};
    return Object.entries(a.conversations).forEach(([c, i]) => {
      new Date(i.expiresAt) > n && (o[c] = i);
    }), { conversations: o };
  } catch (t) {
    return console.error("Failed to load conversation history:", t), { conversations: {} };
  }
}
function le(t) {
  try {
    localStorage.setItem(me, JSON.stringify(t));
  } catch (a) {
    console.error("Failed to save conversation history:", a);
  }
}
function Je() {
  const t = Be(), [a, n] = j(() => {
    const m = J().conversations[t];
    return m ? m.messages.map((g) => ({
      ...g,
      timestamp: new Date(g.timestamp)
    })) : [];
  });
  N(() => {
    if (a.length === 0) return;
    const u = J();
    u.conversations[t] = {
      id: t,
      messages: a,
      lastUpdated: (/* @__PURE__ */ new Date()).toISOString(),
      expiresAt: He()
    }, le(u);
  }, [a, t]);
  const o = (u) => {
    n((m) => [...m, u]);
  }, c = () => {
    n([]);
  }, i = () => {
    const u = J();
    delete u.conversations[t], le(u), n([]);
  }, l = a.length;
  return {
    messages: a,
    addMessage: o,
    clearMessages: c,
    deleteConversation: i,
    messageCount: l
  };
}
function Xe() {
  const [t, a] = j({
    text: "",
    hasSelection: !1
  });
  N(() => {
    function c() {
      const i = window.getSelection();
      if (!i) {
        a({ text: "", hasSelection: !1 });
        return;
      }
      const l = i.toString().trim();
      l.length >= 3 ? a({
        text: l,
        hasSelection: !0
      }) : a({
        text: "",
        hasSelection: !1
      });
    }
    return document.addEventListener("selectionchange", c), () => {
      document.removeEventListener("selectionchange", c);
    };
  }, []);
  const n = () => {
    a({ text: "", hasSelection: !1 }), window.getSelection()?.removeAllRanges();
  }, o = (c = 50) => t.text.length <= c ? t.text : `${t.text.substring(0, c)}...`;
  return {
    selectedText: t.text,
    hasSelection: t.hasSelection,
    clearSelection: n,
    getTruncatedText: o
  };
}
const Ke = [
  { title: "Week 1: Introduction to Physical AI", url: "/docs/week-1" },
  { title: "Week 2: Robot Kinematics", url: "/docs/week-2" },
  { title: "Week 3: ROS 2 Fundamentals", url: "/docs/week-3" },
  { title: "Week 4: NVIDIA Isaac Platform", url: "/docs/week-4" },
  { title: "Week 5: Simulation with Gazebo", url: "/docs/week-5" }
], ue = [
  "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides services like hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.",
  "Physical AI combines artificial intelligence with physical embodiment, enabling robots and autonomous systems to interact with and learn from the real world.",
  "NVIDIA Isaac is a platform for developing, testing, and deploying AI-based robots. It includes simulation tools, perception models, and deployment capabilities.",
  "Gazebo is a robotics simulator that provides accurate physics simulation, sensor simulation, and integration with ROS for testing robot software.",
  "Humanoid robotics involves designing robots with human-like characteristics to operate in environments designed for humans."
];
async function Ze(t) {
  const a = 500 + Math.random() * 1e3;
  if (await new Promise((m) => setTimeout(m, a)), Math.random() < 0.05)
    throw new Error("Failed to connect to the assistant. Please try again.");
  const n = ue[Math.floor(Math.random() * ue.length)], o = Math.random() > 0.5 ? 2 : 1, i = [...Ke].sort(() => Math.random() - 0.5).slice(0, o);
  let l = n;
  if (t.selectedText) {
    const m = t.selectedText.substring(0, 50), g = t.selectedText.length > 50 ? "..." : "";
    l = `Regarding "${m}${g}": ${n}`;
  }
  return {
    message: {
      id: `msg-${Date.now()}`,
      role: "assistant",
      content: l,
      timestamp: /* @__PURE__ */ new Date(),
      sources: i
    }
  };
}
const X = {
  id: "welcome",
  role: "system",
  content: `👋 Hello! I'm your Physical AI & Robotics assistant. Ask me anything about:
• ROS 2 and robot control
• NVIDIA Isaac platform
• Humanoid robotics
• Simulation with Gazebo

💡 Tip: Select any text on the page and ask me about it!`,
  timestamp: /* @__PURE__ */ new Date()
};
function Qe({ isOpen: t, onClose: a }) {
  const {
    messages: n,
    addMessage: o,
    clearMessages: c
  } = Je(), {
    selectedText: i,
    hasSelection: l,
    clearSelection: u,
    getTruncatedText: m
  } = Xe(), [g, p] = j([X]), [y, T] = j(""), [_, C] = j(!1), [P, k] = j(null), I = ae(null), w = ae(null);
  N(() => {
    n.length > 0 && p([X, ...n]);
  }, []), N(() => {
    I.current?.scrollIntoView({ behavior: "smooth" });
  }, [g]), N(() => {
    w.current && (w.current.style.height = "auto", w.current.style.height = `${Math.min(w.current.scrollHeight, 120)}px`);
  }, [y]);
  const $ = async () => {
    if (!y.trim()) return;
    const b = y.trim(), x = 2e3;
    if (b.length > x) {
      k(`Message too long. Maximum ${x} characters allowed.`);
      return;
    }
    k(null);
    const S = {
      id: `user-${Date.now()}`,
      role: "user",
      content: b,
      timestamp: /* @__PURE__ */ new Date()
    }, W = [...g, S];
    p(W), o(S), T(""), C(!0);
    try {
      const R = await Ze({
        message: S.content,
        selectedText: l ? i : void 0
      });
      p((E) => [...E, R.message]), o(R.message), l && u();
    } catch (R) {
      const E = R instanceof Error ? R.message : "An unexpected error occurred";
      k(E);
      const D = {
        id: `error-${Date.now()}`,
        role: "assistant",
        content: `❌ ${E}`,
        timestamp: /* @__PURE__ */ new Date()
      };
      p((q) => [...q, D]);
    } finally {
      C(!1);
    }
  }, V = (b) => {
    b.key === "Enter" && !b.shiftKey && (b.preventDefault(), $());
  }, F = () => {
    c(), p([X]), k(null);
  }, L = () => {
    l && (T(`Can you explain: "${i}"`), w.current?.focus());
  };
  return t ? /* @__PURE__ */ s.jsxs("div", { className: "chatbot-panel", children: [
    /* @__PURE__ */ s.jsxs("div", { className: "chatbot-header", children: [
      /* @__PURE__ */ s.jsxs("div", { className: "chatbot-header__left", children: [
        /* @__PURE__ */ s.jsx(Te, { size: 24 }),
        /* @__PURE__ */ s.jsxs("div", { className: "chatbot-header__text", children: [
          /* @__PURE__ */ s.jsx("h3", { children: "Physical AI Assistant" }),
          /* @__PURE__ */ s.jsx("span", { children: "Ask me about the book" })
        ] })
      ] }),
      /* @__PURE__ */ s.jsx(
        "button",
        {
          className: "chatbot-header__close",
          onClick: a,
          "aria-label": "Close chat",
          children: /* @__PURE__ */ s.jsx(ie, { size: 24 })
        }
      )
    ] }),
    /* @__PURE__ */ s.jsxs("div", { className: "chatbot-messages", role: "log", "aria-live": "polite", children: [
      g.map((b) => /* @__PURE__ */ s.jsx(Ue, { message: b }, b.id)),
      _ && /* @__PURE__ */ s.jsxs("div", { className: "chatbot-typing", role: "status", "aria-label": "Assistant is typing", children: [
        /* @__PURE__ */ s.jsx("span", { className: "chatbot-typing__dot" }),
        /* @__PURE__ */ s.jsx("span", { className: "chatbot-typing__dot" }),
        /* @__PURE__ */ s.jsx("span", { className: "chatbot-typing__dot" })
      ] }),
      P && /* @__PURE__ */ s.jsxs("div", { className: "chatbot-error", role: "alert", children: [
        /* @__PURE__ */ s.jsx(Ne, { size: 16 }),
        /* @__PURE__ */ s.jsx("span", { children: P })
      ] }),
      /* @__PURE__ */ s.jsx("div", { ref: I })
    ] }),
    l && /* @__PURE__ */ s.jsxs("div", { className: "chatbot-selected-text", children: [
      /* @__PURE__ */ s.jsx($e, { size: 16 }),
      /* @__PURE__ */ s.jsxs("span", { className: "chatbot-selected-text__preview", children: [
        'Selected: "',
        m(),
        '"'
      ] }),
      /* @__PURE__ */ s.jsx(
        "button",
        {
          className: "chatbot-selected-text__ask",
          onClick: L,
          "aria-label": "Ask about selected text",
          children: "Ask about this"
        }
      ),
      /* @__PURE__ */ s.jsx(
        "button",
        {
          className: "chatbot-selected-text__clear",
          onClick: u,
          "aria-label": "Clear selection",
          children: /* @__PURE__ */ s.jsx(ie, { size: 14 })
        }
      )
    ] }),
    /* @__PURE__ */ s.jsxs("div", { className: "chatbot-input", children: [
      /* @__PURE__ */ s.jsxs("div", { className: "chatbot-input__main", children: [
        /* @__PURE__ */ s.jsx(
          "textarea",
          {
            ref: w,
            className: "chatbot-input__textarea",
            value: y,
            onChange: (b) => T(b.target.value),
            onKeyDown: V,
            placeholder: "Ask about Physical AI...",
            "aria-label": "Type your question",
            rows: 1,
            disabled: _
          }
        ),
        /* @__PURE__ */ s.jsx(
          "button",
          {
            className: "chatbot-input__send",
            onClick: $,
            disabled: !y.trim() || _,
            "aria-label": "Send message",
            children: /* @__PURE__ */ s.jsx(ze, { size: 20 })
          }
        )
      ] }),
      /* @__PURE__ */ s.jsxs("div", { className: "chatbot-input__actions", children: [
        /* @__PURE__ */ s.jsxs(
          "button",
          {
            className: "chatbot-input__action",
            onClick: F,
            "aria-label": "Clear chat history",
            children: [
              /* @__PURE__ */ s.jsx(Ve, { size: 14 }),
              /* @__PURE__ */ s.jsx("span", { children: "Clear chat" })
            ]
          }
        ),
        /* @__PURE__ */ s.jsxs(
          "button",
          {
            className: "chatbot-input__action",
            onClick: () => {
              const b = g.filter((x) => x.role === "assistant" && x.sources).flatMap((x) => x.sources || []);
              b.length > 0 && alert(`${b.length} source(s) referenced in this conversation`);
            },
            "aria-label": "View sources",
            children: [
              /* @__PURE__ */ s.jsx(Se, { size: 14 }),
              /* @__PURE__ */ s.jsx("span", { children: "View sources" })
            ]
          }
        )
      ] })
    ] })
  ] }) : null;
}
function tt(t) {
  const { position: a = "bottom-right", primaryColor: n = "#2563eb" } = t, [o, c] = j(!1), [i, l] = j(!0);
  N(() => {
    const p = setTimeout(() => {
      l(!1);
    }, 6e3);
    return () => clearTimeout(p);
  }, []);
  const u = H(() => {
    c((p) => !p), l(!1);
  }, []), m = H(() => {
    c(!1);
  }, []), g = H(
    (p) => {
      p.key === "Escape" && o && c(!1);
    },
    [o]
  );
  return /* @__PURE__ */ s.jsxs(
    "div",
    {
      className: `chatbot-widget chatbot-widget--${a}`,
      onKeyDown: g,
      style: { "--chatbot-primary": n },
      children: [
        /* @__PURE__ */ s.jsx(
          We,
          {
            isOpen: o,
            showPulse: i,
            onClick: u
          }
        ),
        /* @__PURE__ */ s.jsx(Qe, { isOpen: o, onClose: m })
      ]
    }
  );
}
export {
  tt as ChatbotWidget
};
